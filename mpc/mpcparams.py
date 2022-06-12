import params

class WalkParams(params.WalkParams):

    # Inherited from params.WalkParams
    stateTerminalWeight = 20
    saveFile = '/tmp/mpc.npy'

    # New parameters
    T_START = 30
    T_SINGLE = 50
    T_DOUBLE = 11
    T_END = 30
    Tmpc = 100
    

