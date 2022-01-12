def dictionary(dictionary):

    d={
    "ASV_MODE": {
        "0":  "MANUAL",
        "1":  "CIRCLE",
        "2":  "STABILIZE",
        "3":  "TRAINING",
        "4":  "ACRO",
        "5":  "FBWA",
        "6":  "FBWB",
        "7":  "CRUISE",
        "8":  "AUTOTUNE",
        "10": "AUTO",
        "11": "RTL",
        "12": "LOITER",
        "13": "TAKEOFF",
        "14": "AVOID_ADSB",
        "15": "GUIDED",
        "16": "INITIALISING",
        "17": "QSTABILIZE",
        "18": "QHOVER",
        "19": "QLOITER",
        "20": "QLAND",
        "21": "QRTL",
        "22": "QAUTOTUNE",
        "23": "QACRO",
        "24": "THERMAL"
        },
    "MISSION_MODE": {
        "0": "STANDBY",
        "1": "GUIDED",
        "2": "MANUAL",
        "3": "SIMPLE",
        "4": "RTL"
        }
    }
    return d[dictionary]
#TODO: check  pymavlink.mavutil.mavlink.
# mavlink dictionary includes this and other useful info