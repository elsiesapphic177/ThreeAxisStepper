idf_component_register(
    SRCS 
        "mathStructure.cpp"  
        "MotionProfile.cpp"
        "PulseGenerator.cpp"
        "Stepper.cpp"
        "Ruler.cpp"
        "SpeedPlanner.cpp"
        "Dispatcher.cpp"
        "Planner.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)
