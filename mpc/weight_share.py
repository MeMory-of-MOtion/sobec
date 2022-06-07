import numpy as np

def switch_tanh(x):
    return 0 if x<=0 else 1 if x>=1 else (np.tanh(-1/x+1/(1-x))+1)/2
def switch_linear(x):
    return x

def weightShareSmoothProfile(contactPattern,duration,switch=switch_linear,verbose=False):

    contactImportance = np.array(contactPattern,dtype=np.float64)
    nbc=np.sum(contactImportance,1)
    for ck in contactImportance.T: ck/=nbc

    T = len(contactPattern)-1
    Ttrans = duration
    trans = np.array([ switch(x) for x in np.arange(1,Ttrans+1)/(Ttrans+1) ])

    for t in range(1,T):
        if np.any(np.logical_and(np.logical_not(contactPattern[t-1]),contactPattern[t])):
            for k,cid in enumerate(contactPattern[t]):
                contactImportance[t:t+Ttrans,k] = contactImportance[t-1,k]*(1-trans)+contactImportance[t,k]*trans
                if verbose and contactImportance[t-1,k] == 0:
                    print(f'Create {t}:{cid}')
    for t in reversed(range(1,T)):
        if np.any(np.logical_and(np.logical_not(contactPattern[t]),contactPattern[t-1])):
            for k,cid in enumerate(contactPattern[t]):
                contactImportance[t-Ttrans:t,k] = contactImportance[t,k]*(trans)+contactImportance[t-1,k]*(1-trans)
                if verbose and contactImportance[t,k] == 0:
                    print(f'Break {t}:{cid}')

    return contactImportance

if __name__ == "__main__":
    import matplotlib.pylab as plt; plt.ion()
    contactPattern = [] \
        + [ [ 1,1 ] ] * 30 \
        + [ [ 1,0 ] ] * 50  \
        + [ [ 1,1 ] ] * 20 \
        + [ [ 1,1 ] ]
    contactIds = [ 34,48 ]

    contactImportance = weightShareSmoothProfile(contactPattern ,10,verbose=True)
    assert( np.linalg.norm(np.sum(contactImportance,1)-1)<=1e-6)
    plt.plot(contactImportance)

    contactImportance = weightShareSmoothProfile(contactPattern ,10,
                                                 switch=switch_tanh,verbose=True)
    assert( np.linalg.norm(np.sum(contactImportance,1)-1)<=1e-6)
    plt.plot(contactImportance)
