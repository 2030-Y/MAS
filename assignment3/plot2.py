import csv
import glob
import os
import ast
import math
import sys
import matplotlib.pyplot as plt


def read_metrics_csv(path: str):
    cols = {
        "agent_id": [],
        "t": [],
        "x": [],
        "y": [],
        "dmin": [],
        "episode_return": [],
        "ema_return": [],
        "recent_mean": [],
        "collected": [],
        "since_last_collect": [],
        "pos_regret_sum": [],
        "regret_max": [],
        "action_dist": [],
    }

    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Skip rows missing agent_id or t
            if row.get("agent_id") is None or row.get("t") is None:
                continue
            try:
                cols["agent_id"].append(int(float(row.get("agent_id"))))
                cols["t"].append(int(float(row.get("t"))))
                cols["x"].append(float(row.get("x")))
                cols["y"].append(float(row.get("y")))
                cols["dmin"].append(float(row.get("dmin")))
                cols["episode_return"].append(float(row.get("episode_return")))
                cols["ema_return"].append(float(row.get("ema_return")))
                cols["recent_mean"].append(float(row.get("recent_mean")))
                cols["collected"].append(int(float(row.get("collected"))))
                cols["since_last_collect"].append(int(float(row.get("since_last_collect"))))
                cols["pos_regret_sum"].append(float(row.get("pos_regret_sum")))
                cols["regret_max"].append(float(row.get("regret_max")))
                cols["action_dist"].append(ast.literal_eval(row.get("action_dist")))
            except (TypeError, ValueError):
                # skip malformed row
                continue

    # sort by t if present
    if cols["t"]:
        order = sorted(range(len(cols["t"])), key=lambda i: (cols["t"][i] is None, cols["t"][i]))
        for k in list(cols.keys()):
            cols[k] = [cols[k][i] for i in order]

    # explode action_dist into action_0..N
    max_len = 0
    for v in cols["action_dist"]:
        if isinstance(v, (list, tuple)):
            max_len = max(max_len, len(v))
    for i in range(max_len):
        arr = []
        for v in cols["action_dist"]:
            if isinstance(v, (list, tuple)) and len(v) > i:
                arr.append(float(v[i]))
            else:
                arr.append(None)
        cols[f"action_{i}"] = arr

    return cols


def mask_none(x_list, y_list):
    xx, yy = [], []
    for x, y in zip(x_list, y_list):
        if x is None or y is None:
            continue
        xx.append(x)
        yy.append(y)
    return xx, yy


def mean_std_ignore_none(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None
    m = sum(vals) / float(len(vals))
    var = sum((v - m) ** 2 for v in vals) / float(len(vals))
    return m, math.sqrt(var)


def build_mean_std_series(agent_data, field):
    t = next(iter(agent_data.values()))["t"]
    mean_list, std_list = [], []
    for idx in range(len(t)):
        values = []
        for cols in agent_data.values():
            if field not in cols:
                continue
            if idx < len(cols[field]):
                values.append(cols[field][idx])
        m, s = mean_std_ignore_none(values)
        mean_list.append(m)
        std_list.append(s)
    return t, mean_list, std_list


def last_valid_value(series):
    for v in reversed(series):
        if v is not None:
            return v
    return None


def topk_agents_by_final(agent_data, field, k=5):
    ranked = []
    for aid, cols in agent_data.items():
        if field not in cols:
            continue
        v = last_valid_value(cols[field])
        if v is None:
            continue
        ranked.append((aid, v))
    ranked.sort(key=lambda x: x[1], reverse=True)
    return [aid for aid, _ in ranked[:k]]


# ---------------- Plotting ----------------
def plot_mean_std(agent_data, field, title, ylabel, outpath):
    t, mean_list, std_list = build_mean_std_series(agent_data, field)
    x, y_mean = mask_none(t, mean_list)
    _, y_std = mask_none(t, std_list)
    if not x:
        return
    y_std_aligned = []
    for tt, mm, ss in zip(t, mean_list, std_list):
        if tt is None or mm is None or ss is None:
            continue
        y_std_aligned.append(ss)

    plt.figure()
    plt.plot(x, y_mean, label="mean", linewidth=2)
    lower = [m - s for m, s in zip(y_mean, y_std_aligned)]
    upper = [m + s for m, s in zip(y_mean, y_std_aligned)]
    plt.fill_between(x, lower, upper, alpha=0.25, label="±1 std")

    plt.title(title)
    plt.xlabel("t")
    plt.ylabel(ylabel)
    handles, labels = plt.gca().get_legend_handles_labels()
    if labels:
        plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def plot_topk_compare(agent_data, field, title, ylabel, outpath, k=5, faint_alpha=0.15):
    top_ids = set(topk_agents_by_final(agent_data, field, k=k))

    plt.figure()
    for aid, cols in agent_data.items():
        if field not in cols:
            continue
        x, y = mask_none(cols["t"], cols[field])
        if not x:
            continue
        if aid in top_ids:
            plt.plot(x, y, linewidth=2, label=f"agent {aid}")
        else:
            plt.plot(x, y, alpha=faint_alpha)

    plt.title(title)
    plt.xlabel("t")
    plt.ylabel(ylabel)
    handles, labels = plt.gca().get_legend_handles_labels()
    if labels:
        plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def find_max_action_index(agent_data):
    max_idx = -1
    for cols in agent_data.values():
        for k in cols.keys():
            if k.startswith("action_"):
                suf = k.split("_", 1)[1]
                if suf.isdigit():
                    max_idx = max(max_idx, int(suf))
    return max_idx


def run_all_plots(agent_data, outdir_mean_std, outdir_top5):
    os.makedirs(outdir_mean_std, exist_ok=True)
    os.makedirs(outdir_top5, exist_ok=True)

    scalar_fields = [
        ("episode_return", "Episode return", "episode_return"),
        ("ema_return", "EMA return", "ema_return"),
        ("recent_mean", "Recent mean reward", "recent_mean"),
        ("dmin", "Distance to nearest reward dmin", "dmin"),
        ("collected", "Collected count", "collected"),
        ("pos_regret_sum", "Positive regret sum", "pos_regret_sum"),
        ("regret_max", "Regret max", "regret_max"),
    ]

    for field, title, ylabel in scalar_fields:
        plot_mean_std(
            agent_data, field,
            f"{title} (mean ± std over agents)", ylabel,
            os.path.join(outdir_mean_std, f"{field}_mean_std.png"),
        )
        plot_topk_compare(
            agent_data, field,
            f"{title} (Top-5 agents highlighted)", ylabel,
            os.path.join(outdir_top5, f"{field}_top5.png"),
            k=5
        )

    action_names = ["forward", "fwd_left", "fwd_right", "rot_left", "rot_right"]
    max_idx = find_max_action_index(agent_data)
    if max_idx >= 0:
        for i in range(max_idx + 1):
            field = f"action_{i}"
            name = action_names[i] if i < len(action_names) else field
            plot_mean_std(
                agent_data, field,
                f"Action prob: {name} (mean ± std over agents)", "probability",
                os.path.join(outdir_mean_std, f"action_{name}_mean_std.png"),
            )
            plot_topk_compare(
                agent_data, field,
                f"Action prob: {name} (Top-5 agents highlighted)", "probability",
                os.path.join(outdir_top5, f"{field}_top5.png"),
                k=5
            )


def main():
    runs = ["results_5agents", "results_10agents", "results_20agents"]
    for base in runs:
        files = sorted(glob.glob(os.path.join(base, "rm_metrics_agent_*.csv")))
        if not files:
            print(f"Skip {base}: no CSVs found")
            continue

        agent_data = {}
        for f in files:
            cols = read_metrics_csv(f)
            if not cols["agent_id"] or not cols["t"]:
                # skip empty/malformed file
                continue
            aid = cols["agent_id"][0] if cols.get("agent_id") and cols["agent_id"] else os.path.splitext(os.path.basename(f))[0]
            agent_data[aid] = cols

        if not agent_data:
            print(f"Skip {base}: no valid data")
            continue

        outdir = os.path.join(base, "plots")
        out_mean_std = os.path.join(outdir, "mean_std")
        out_top5 = os.path.join(outdir, "top5")
        run_all_plots(agent_data, out_mean_std, out_top5)
        print(f"[{base}] Done. Mean±std: {out_mean_std}/ ; Top-5: {out_top5}/")


if __name__ == "__main__":
    main()