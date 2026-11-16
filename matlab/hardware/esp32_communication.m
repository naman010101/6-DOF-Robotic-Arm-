%% ---------------- Serial ----------------

COM = "COM9";
BAUD = 115200;

try
    esp = serialport(COM,BAUD);
    configureTerminator(esp,"LF");
    flush(esp);

    disp("ESP32 Connected")

catch

    esp = [];

    warning("ESP32 Not Connected")

end