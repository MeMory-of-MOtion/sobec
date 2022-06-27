import numpy as np
from numpy.linalg import norm


def logsToDict(logs):
    dlogs = {}
    for k in dir(logs):
        if k[:2] == "__":
            continue
        dlogs[k] = np.array(getattr(logs, k))
    return dlogs


def save(dlogs: dict, filename="/tmp/logs.npy", verbose=True):
    with open(filename, "wb") as f:
        np.save(f, dlogs, allow_pickle=True)
    print('Logs save in "%s"!' % filename)


def load(filename="/tmp/logs.npy"):
    return np.load(filename, allow_pickle=True)[()]


def checkGitRefs(
    logs, git_filename, save_filename="/tmp/logs.npy", saveIfFailed=True, verbose=True
):
    dlogs = logsToDict(logs)
    gitlogs = load(git_filename)
    res = areLogsEquals(dlogs, gitlogs, verbose=verbose)
    if not res:
        if verbose:
            print("***")
            print('*** Logs are dissimilar to git reference "%s"!' % git_filename)
            print("***")
        if saveIfFailed:
            save(dlogs, save_filename, verbose)
    return res


def areLogsEquals(dlogs1, dlogs2, th=1e-4, verbose=False):
    res = True
    histLength = min(len(dlogs1["costs"]), len(dlogs2["costs"]))
    if norm(dlogs1["costs"][:histLength] - dlogs2["costs"][:histLength]) > th:
        res = False
        if verbose:
            print("Cost backlogs are not the same")
    if norm(dlogs1["stops"][:histLength] - dlogs2["stops"][:histLength]) > th:
        res = False
        if verbose:
            print("Stop backlogs are not the same")
    if norm(dlogs1["xs"][-1, :] - dlogs2["xs"][-1, :]) > th:
        res = False
        if verbose:
            print("Optimal state trajs are not the same")
    if norm(dlogs1["us"][-1, :] - dlogs2["us"][-1, :]) > th:
        res = False
        if verbose:
            print("Optimal control trajs are not the same")
    return res
