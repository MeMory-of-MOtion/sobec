import numpy as np


def switch_tanh(x):
    return (
        0
        if x <= 0
        else 1
        if x >= 1
        else (np.tanh(-1.0 / x + 1.0 / (1.0 - x)) + 1.0) / 2.0
    )


def switch_linear(x):
    return x


def weightShareSmoothProfile(
    contactPattern, duration, switch=switch_linear, verbose=False
):

    contactImportance = np.array(contactPattern, dtype=np.float64)
    nbc = np.sum(contactImportance, 1)
    for ck in contactImportance.T:
        ck /= nbc

    T = len(contactPattern) - 1
    Ttrans = duration
    trans = np.array([switch(x) for x in np.arange(1, Ttrans + 1.0) / (Ttrans + 1.0)])

    for t in range(1, T):
        if np.any(
            np.logical_and(np.logical_not(contactPattern[t - 1]), contactPattern[t])
        ):
            for k, cid in enumerate(contactPattern[t]):
                contactImportance[t : t + Ttrans, k] = (
                    contactImportance[t - 1, k] * (1 - trans)
                    + contactImportance[t, k] * trans
                )
                if verbose and contactImportance[t - 1, k] == 0:
                    print("Create %s:%s" % (t, cid))
    for t in reversed(range(1, T)):
        if np.any(
            np.logical_and(np.logical_not(contactPattern[t]), contactPattern[t - 1])
        ):
            for k, cid in enumerate(contactPattern[t]):
                contactImportance[t - Ttrans : t, k] = contactImportance[t, k] * (
                    trans
                ) + contactImportance[t - 1, k] * (1 - trans)
                if verbose and contactImportance[t, k] == 0:
                    print("Break %s:%s" % (t, cid))

    return contactImportance


def computeBestTransitionDuration(contactPattern, maxTransitionDuration):
    """
    Compute the maximal transition duration to avoid bad effects in
    weightShareSmoothProfile().
    """
    contactState = []
    dur = mindur = len(contactPattern)
    for t, s in enumerate(contactPattern):
        dur += 1
        if s != contactState:
            contactState = s
            mindur = min(mindur, dur)
            dur = 0

    # Select the smoothing transition to be smaller than half of the minimal duration.
    transitionDuration = min((mindur - 1) // 2, maxTransitionDuration)
    return transitionDuration


def computeReferenceForces(
    contactPattern, robotweight, transitionDuration=-1, maxTransitionDuration=50
):
    """
    # The force costs are defined using a reference (smooth) force.
    # Search the contact phase of minimal duration (typically double support)
    """
    T = len(contactPattern)

    if transitionDuration < 0:
        transitionDuration = computeBestTransitionDuration(
            contactPattern, maxTransitionDuration
        )

    # Compute contact importance, ie how much of the weight should be supported by each
    # foot at each time.
    contactImportance = weightShareSmoothProfile(
        contactPattern, transitionDuration, switch=switch_linear
    )
    # Contact reference forces are set to contactimportance*weight
    weightReaction = np.array([0, 0, robotweight, 0, 0, 0])
    referenceForces = [
        [
            weightReaction * contactImportance[t, k]
            for k, p in enumerate(contactPattern[t])
        ]
        for t in range(T)
    ]

    # Take care, we suppose here that foot normal is vertical.
    return referenceForces


if __name__ == "__main__":
    import matplotlib.pylab as plt

    plt.ion()
    contactPattern = [] + [[1, 1]] * 30 + [[1, 0]] * 50 + [[1, 1]] * 20 + [[1, 1]]
    contactIds = [34, 48]

    contactImportance = weightShareSmoothProfile(contactPattern, 10, verbose=True)
    assert np.linalg.norm(np.sum(contactImportance, 1) - 1) <= 1e-6
    plt.plot(contactImportance)

    contactImportance = weightShareSmoothProfile(
        contactPattern, 10, switch=switch_tanh, verbose=True
    )
    assert np.linalg.norm(np.sum(contactImportance, 1) - 1) <= 1e-6
    plt.plot(contactImportance)

    plt.figure("Reference force for weight=500")
    ref = computeReferenceForces(contactPattern, 500)
    plt.plot([np.concatenate(f)[[2, 6 + 2]] for f in ref])
