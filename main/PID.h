typedef struct _PIDdata {
    uint16_t PIDinput_prev;
    uint16_t PIDsetpoint;

    // PID factors
    float PIDkP;
    float PIDkI;
    float PIDkD;

    // PID terms
    float PIDPerr;
    float PIDIerr;
    float PIDDerr;

    // PID terms limits
    float PIDPerrmin;
    float PIDPerrmax;
    float PIDIerrmin;
    float PIDIerrmax;
} PIDdata;