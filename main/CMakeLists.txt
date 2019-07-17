set(COMPONENT_SRCS 
    "main.c"
    "variables.c"
    "battery.c"
    "encoder.c"
    "indicator.c"
    "motion.c"
    "wall_detector.c"
    "parameters.c"
    "controller.c"
    )
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
