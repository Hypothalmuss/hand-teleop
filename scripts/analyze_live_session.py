#!/usr/bin/env python3
"""Analyse a recorded live session (scripts/live_session.py) into results/.

    python3 scripts/analyze_live_session.py recordings/live_session --out results/live_session

Writes report.md, metrics.json, attempts.csv, latency.csv and plots.
"""

import argparse
import csv
import json
import os
import pickle

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from ur5e_2f85_mujoco import load_model_names  # noqa: E402
from ur5e_2f85_mujoco.config import load_config  # noqa: E402
from ur5e_2f85_mujoco.kinematics import Kinematics  # noqa: E402

AXES = "xyz"
SWEEP_AXIS = {"sweep_lr": 1, "sweep_ud": 2, "sweep_fb": 0}
SETTLE_S = 2.0  # ignore the first seconds of each step


def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float("nan")


def rows(log, key, step=None, prefix=None, col=2):
    out = [r for r in log[key] if (step is None or r[col] == step)
           and (prefix is None or str(r[col]).startswith(prefix))]
    return out


def after_settle(rs, t_idx=0):
    if not rs:
        return rs
    t0 = rs[0][t_idx]
    return [r for r in rs if r[t_idx] - t0 >= SETTLE_S]


def detrended_std(a):
    a = np.asarray(a, float)
    if len(a) < 3:
        return float("nan")
    t = np.arange(len(a))
    return float(np.std(a - np.polyval(np.polyfit(t, a, 1), t)))


def lag_ms(t_ref, x_ref, t_sig, x_sig):
    """Delay of x_sig behind x_ref by cross-correlation on a common 200 Hz grid."""
    if len(t_ref) < 20 or len(t_sig) < 20:
        return float("nan")
    t0, t1 = max(t_ref[0], t_sig[0]), min(t_ref[-1], t_sig[-1])
    grid = np.arange(t0, t1, 0.005)
    a = np.interp(grid, t_ref, x_ref) - np.mean(x_ref)
    b = np.interp(grid, t_sig, x_sig) - np.mean(x_sig)
    lags = np.arange(0, 100)  # 0..500 ms
    err = [np.mean((a[: len(a) - k] - b[k:]) ** 2) for k in lags]
    return float(lags[int(np.argmin(err))] * 5.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir")
    ap.add_argument("--out", default="results/live_session")
    args = ap.parse_args()
    with open(os.path.join(args.session_dir, "session.pkl"), "rb") as f:
        log = pickle.load(f)
    os.makedirs(args.out, exist_ok=True)
    ws = load_config("workspace.yaml")
    filt = load_config("filters.yaml")
    names = load_model_names()
    m = {}  # metrics
    sim = [r[1] for r in log["cmd"]]
    if len(sim) > 2:
        dur = sim[-1] - sim[0]
        m["logged_rates_hz"] = {k: round(len(log[k]) / dur, 1) for k in ("cmd", "js", "ee",
                                                                          "hand", "target")}

    # ---------------- hand tracking ----------------
    hand = log["hand"]
    fps = np.array([r[10] for r in hand if r[10] > 0])
    m["hand_fps_median"] = float(np.median(fps)) if len(fps) else float("nan")
    m["right_present_pct"] = float(100 * np.mean([r[3] for r in hand])) if hand else 0.0

    # ---------------- still hold ----------------
    W, H = 1920, 1080
    still_t = after_settle(rows(log, "target", "still", col=3))
    still_t = [r for r in still_t if r[8]]
    still_e = after_settle(rows(log, "ee", "still"))
    st = {}
    if still_t:
        tp = np.array([r[4:7] for r in still_t])
        st["target_std_mm"] = [1000 * detrended_std(tp[:, i]) for i in range(3)]
        st["target_p2p_mm"] = [1000 * float(np.ptp(tp[:, i])) for i in range(3)]
    if still_e:
        ep = np.array([r[3:6] for r in still_e])
        st["tcp_std_mm"] = [1000 * detrended_std(ep[:, i]) for i in range(3)]
    sh = after_settle([r for r in rows(log, "hand", "still") if r[3]])
    sr = after_settle([r for r in rows(log, "hand_raw", "still") if r[3]])
    if sh and sr:
        f9 = np.array([r[4][27:29] for r in sh]) * [W, H]
        r9 = np.array([r[4][27:29] for r in sr]) * [W, H]
        st["lm9_px_std_filtered"] = [detrended_std(f9[:, i]) for i in range(2)]
        st["lm9_px_std_raw"] = [detrended_std(r9[:, i]) for i in range(2)]
        fp = np.array([r[5] for r in sh])
        rp = np.array([r[5] for r in sr])
        st["palm_scale_rel_std_pct_filtered"] = 100 * detrended_std(fp) / np.mean(fp)
        st["palm_scale_rel_std_pct_raw"] = 100 * detrended_std(rp) / np.mean(rp)
        st["open_flag_toggles"] = int(np.count_nonzero(np.diff([r[6] for r in sh])))
    m["still"] = st

    # ---------------- sweeps ----------------
    sweeps = {}
    for step, ax in SWEEP_AXIS.items():
        tr = [r for r in rows(log, "target", step, col=3) if r[8]]
        ee = rows(log, "ee", step)
        if len(tr) < 10:
            continue
        tp = np.array([r[4:7] for r in tr])
        tt = np.array([r[1] for r in tr])
        box = ws["box"][AXES[ax]]
        span = float(np.ptp(tp[:, ax]))
        others = [i for i in range(3) if i != ax]
        v = np.linalg.norm(np.diff(tp, axis=0), axis=1) / np.maximum(np.diff(tt), 1e-3)
        lat = [r[6] for r in rows(log, "latency", step, col=1)]
        et = np.array([r[1] for r in ee])
        ep = np.array([r[3:6] for r in ee])
        sweeps[step] = {
            "axis": AXES[ax],
            "range_used_pct_of_box": 100 * span / (box[1] - box[0]),
            "cross_axis_std_mm": [1000 * float(np.std(tp[:, i])) for i in others],
            "target_speed_p95_mps": pct(v, 95),
            "tracking_error_rms_mm": 1000 * float(np.sqrt(np.mean(np.square(lat)))) if lat
            else float("nan"),
            "tcp_lag_behind_target_ms": lag_ms(tt, tp[:, ax], et, ep[:, ax]) if len(ee) else
            float("nan"),
        }
    m["sweeps"] = sweeps

    # ---------------- clutch ----------------
    tr = rows(log, "target", "clutch", col=3)
    ee = rows(log, "ee", "clutch")
    jumps, n_edges = [], 0
    if tr and ee:
        et = np.array([r[1] for r in ee])  # sim header stamps on both streams
        ep = np.array([r[3:6] for r in ee])
        for a, b in zip(tr[:-1], tr[1:]):
            if b[8] and not a[8]:
                n_edges += 1
                i = int(np.argmin(np.abs(et - b[1])))
                jumps.append(1000 * float(np.linalg.norm(np.array(b[4:7]) - ep[i])))
    m["clutch"] = {"engage_edges": n_edges, "target_minus_tcp_at_engage_mm_max":
                   max(jumps) if jumps else float("nan")}

    # ---------------- gripper ----------------
    cmd = rows(log, "cmd", "gripper")
    js = rows(log, "js", "gripper")
    if cmd and js:
        g = np.array([r[4] for r in cmd])
        gt = np.array([r[0] for r in cmd])
        ap_ = np.array([r[5] for r in js])
        at = np.array([r[0] for r in js])
        m["gripper"] = {"cmd_min": float(g.min()), "cmd_max": float(g.max()),
                        "close_open_cycles": int(np.count_nonzero(np.diff(g > 0.5))) // 2,
                        "aperture_lag_ms": lag_ms(gt, g, at, ap_)}

    # ---------------- e-stop ----------------
    ev = {e["name"]: e for e in log["events"]}
    if "estop_on" in ev and "estop_clear" in ev:
        t_on, t_clear = ev["estop_on"]["wall"], ev["estop_clear"]["wall"]
        during = [r for r in log["cmd"] if t_on + 0.1 < r[0] < t_clear]
        q = np.array([r[3] for r in during]) if during else np.zeros((0, 6))
        js_w = [r for r in log["js"] if t_on < r[0] < min(t_on + 2.0, t_clear)]
        qs = np.array([r[3] for r in js_w])
        tgt_moving = [r for r in log["target"] if t_on < r[0] < t_clear]
        diag = [d for d in log["diag"] if d[1] == "diff_ik_controller" and d[0] > t_on + 2.0]
        m["estop"] = {
            "command_change_during_stop_rad": float(np.abs(q - q[0]).max()) if len(q) else
            float("nan"),
            "joint_travel_2s_after_stop_rad": float(np.abs(qs - qs[0]).max()) if len(qs) else
            float("nan"),
            "cleared_ok": bool(ev["estop_clear"].get("ok")),
            "targets_received_during_stop": len(tgt_moving),
            "ik_reported_braking_rad": float(diag[0][3].get("last_post_estop_braking_rad", "nan"))
            if diag else float("nan"),
            "ik_reported_hold_drift_rad": float(diag[0][3].get("last_post_estop_hold_drift_rad",
                                                               "nan")) if diag else float("nan"),
        }

    # ---------------- latency (whole session) ----------------
    lat = log["latency"]
    hops = {"capture_to_hand": (2, 3), "hand_to_target": (3, 4), "target_to_command": (4, 5),
            "capture_to_command": (2, 5)}
    lat_stats = {}
    with open(os.path.join(args.out, "latency.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step"] + list(hops) + ["tracking_error_mm"])
        for r in lat:
            w.writerow([r[1]] + [round(1000 * (r[b] - r[a]), 1) if r[a] > 0 and r[b] > 0
                                 else "" for a, b in hops.values()]
                       + [round(1000 * r[6], 2)])
    for h, (a, b) in hops.items():
        v = np.array([1000 * (r[b] - r[a]) for r in lat if r[a] > 0 and r[b] > 0])
        lat_stats[h] = {"p50": pct(v, 50), "p95": pct(v, 95), "p99": pct(v, 99), "n": len(v)}
    m["latency_ms"] = lat_stats

    # tracking error while engaged and target speed <= 0.2 m/s
    tg = log["target"]
    speed = {}
    for a, b in zip(tg[:-1], tg[1:]):
        dt = b[1] - a[1]
        if dt > 0:
            speed[b[2]] = float(np.linalg.norm(np.array(b[4:7]) - np.array(a[4:7])) / dt)
    engaged = {r[2]: r[8] for r in tg}
    trk = np.array([r[6] for r in lat if engaged.get(r[2]) and speed.get(r[2], 1e9) <= 0.2])
    m["tracking_error_rms_mm_engaged_le_0p2"] = 1000 * float(np.sqrt(np.mean(trk ** 2))) \
        if len(trk) else float("nan")

    # real-time factor over the session (sim stamp vs wall of ee messages)
    e = log["ee"]
    if len(e) > 100:
        sim = np.array([r[1] for r in e])
        wall = np.array([r[0] for r in e])
        gaps = np.diff(sim) > 0.15  # resets add settle time to the sim clock
        m["rtf"] = float((np.sum(np.diff(sim)[~gaps])) / (np.sum(np.diff(wall)[~gaps])))

    # ---------------- pick and place ----------------
    att = log["attempts"]
    kin = Kinematics()
    r_home = kin.fk(np.array(names["q_home"]))[1]
    finger_axis = float(np.degrees(np.arctan2(r_home[1, 1], r_home[0, 1])))
    for a in att:
        d = (a["cube_yaw_deg"] - finger_axis) % 90.0
        a["yaw_misalignment_deg"] = round(min(d, 90.0 - d), 1)
    if att:
        with open(os.path.join(args.out, "attempts.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(att[0]))
            w.writeheader()
            w.writerows(att)
        ok = [a for a in att if a["success"]]
        mis = np.array([a["yaw_misalignment_deg"] for a in att])
        suc = np.array([a["success"] for a in att])
        m["pick_place"] = {
            "attempts": len(att), "successes": len(ok),
            "median_time_from_reset_s": float(np.median([a["time_from_reset_s"] for a in ok]))
            if ok else float("nan"),
            "median_time_from_engage_s": float(np.median(
                [a["time_from_engage_s"] for a in ok if a["time_from_engage_s"] is not None]))
            if ok else float("nan"),
            "mean_regrasps": float(np.mean([a["regrasps"] for a in att])),
            "success_rate_misalign_le_20deg": float(np.mean(suc[mis <= 20])) if np.any(mis <= 20)
            else float("nan"),
            "success_rate_misalign_gt_20deg": float(np.mean(suc[mis > 20])) if np.any(mis > 20)
            else float("nan"),
            "n_misalign_le_20deg": int(np.sum(mis <= 20)),
            "outcomes": {o: sum(a["outcome"] == o for a in att)
                         for o in sorted({a["outcome"] for a in att})},
        }

    with open(os.path.join(args.out, "metrics.json"), "w") as f:
        json.dump(m, f, indent=1, default=float)

    # ---------------- plots ----------------
    if "target_std_mm" in st:
        fig, ax = plt.subplots(figsize=(6, 3.5))
        x = np.arange(3)
        ax.bar(x - 0.2, st["target_std_mm"], 0.4, label="target", color="#2a7ab9")
        if "tcp_std_mm" in st:
            ax.bar(x + 0.2, st["tcp_std_mm"], 0.4, label="TCP", color="#d95f02")
        ax.set_xticks(x, ["x (depth)", "y", "z"])
        ax.set_ylabel("jitter std [mm]")
        ax.set_title("Hand held still: position jitter per axis")
        ax.legend()
        ax.grid(axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "still_jitter.png"), dpi=120)
        plt.close(fig)
    fig, axs = plt.subplots(3, 1, figsize=(8, 7))
    for axp, (step, axi) in zip(axs, SWEEP_AXIS.items()):
        tr = [r for r in rows(log, "target", step, col=3)]
        ee = rows(log, "ee", step)
        if not tr:
            continue
        t0 = tr[0][1]
        axp.plot([r[1] - t0 for r in tr], [r[4 + axi] for r in tr], color="#2a7ab9",
                 label="target")
        axp.plot([r[1] - t0 for r in ee], [r[3 + axi] for r in ee], color="#d95f02", lw=1,
                 label="TCP")
        axp.set_ylabel(f"{AXES[axi]} [m]")
        axp.set_title(step, fontsize=9)
        axp.grid(alpha=0.3)
    axs[0].legend(loc="upper right")
    axs[-1].set_xlabel("sim time [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "sweeps.png"), dpi=120)
    plt.close(fig)
    v = np.array([1000 * (r[5] - r[2]) for r in lat if r[2] > 0])
    if len(v):
        fig, ax = plt.subplots(figsize=(6, 3.5))
        ax.hist(v, bins=np.arange(0, max(200, v.max() + 10), 10), color="#2a7ab9")
        ax.axvline(np.percentile(v, 50), color="k", ls="--", label=f"p50 {pct(v, 50):.0f} ms")
        ax.axvline(np.percentile(v, 95), color="r", ls="--", label=f"p95 {pct(v, 95):.0f} ms")
        ax.set_xlabel("capture -> command [ms] (includes camera frame age)")
        ax.set_ylabel("samples")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "latency_hist.png"), dpi=120)
        plt.close(fig)
    if att:
        fig, ax = plt.subplots(figsize=(6, 3.5))
        for a in att:
            ax.scatter(a["yaw_misalignment_deg"], a["time_from_reset_s"],
                       c="#1b9e77" if a["success"] else "#d95f02", s=40)
        ax.set_xlabel("cube yaw misalignment vs gripper [deg] (0 = faces aligned, 45 = worst)")
        ax.set_ylabel("time [s]")
        ax.set_title("Pick-and-place attempts (green = success)")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "attempts.png"), dpi=120)
        plt.close(fig)
    print(json.dumps(m, indent=1, default=float))
    print("filters:", filt["landmarks"], filt["palm_scale"])


if __name__ == "__main__":
    main()
