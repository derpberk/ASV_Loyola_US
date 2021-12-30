def str2bool(v):
    """
    Simple conversion of STR argument to boolean value.
    Args:
        v: `str value
    Returns:
        `True/`False depending on the input parameter
    """

    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')