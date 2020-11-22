set(FOLDER_NAME "solver")

COLMAP_ADD_SOURCES(
    admm.h
    bcm_sdp_solver.h
    constrained_l1_solver.h constrained_l1_solver.cpp
    l1_solver.h
    qp_solver.h qp_solver.cpp
    rbr_sdp_solver.h rbr_sdp_solver.cpp
    sdp_solver.h
    solver_option.h
    summary.h
)