def configureMPCWalk(mpc, params):
    mpc.Tmpc = params.Tmpc
    mpc.Tstart = params.Tstart
    mpc.Tdouble = params.Tdouble
    mpc.Tsingle = params.Tsingle
    mpc.Tend = params.Tend
    mpc.DT = params.DT
    mpc.solver_th_stop = params.solver_th_stop
    mpc.vcomRef = params.vcomRef
    mpc.solver_reg_min = params.solver_reg_min
    mpc.solver_maxiter = params.solver_maxiter
