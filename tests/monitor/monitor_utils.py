def get_voltages(run_command_func):
    """Liest Standard-Spannungen aus"""
    voltages = {}
    voltage_types = {
        "core": "VideoCore",
        "sdram_c": "RAM Core",
        "sdram_i": "RAM I/O",
        "sdram_p": "RAM Phy"
    }

    for vtype, description in voltage_types.items():
        volt_str = run_command_func(f"vcgencmd measure_volts {vtype}")
        if "volt=" in volt_str:
            volt = volt_str.replace("volt=", "").replace("V", "")
            voltages[description] = float(volt)

    return voltages
