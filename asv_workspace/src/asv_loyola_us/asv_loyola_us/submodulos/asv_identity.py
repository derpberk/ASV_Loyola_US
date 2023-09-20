from getmac import get_mac_address as gma

def get_asv_identity():

    mac = gma()

    if mac == '70:cf:49:9d:39:1f':
        return 3
    else:
        return 1