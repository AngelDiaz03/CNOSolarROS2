import pvlib

def get_mount_tracker(with_tracker, surface_tilt, surface_azimuth, solpos, axis_tilt, axis_azimuth, max_angle, racking_model='open_rack'):
    '''
    Wrapper that defines a pvlib.pvsystem Mount class and determines
    the module orientation if the mount is in a single axis tracker.
    
    Parameters
    ----------
    with_tracker : bool
        Parameter that checks if the mounting of the array is either on 
        fixed-tilt racking or horizontal single axis tracker.

    surface_tilt : float or list
        Surface tilt angles. The tilt angle is defined as degrees from 
        horizontal (e.g. surface facing up = 0, surface facing 
        horizon = 90).
        
    surface_azimuth : float or list
        Azimuth angle of the module surface. North = 0, East = 90, 
        South = 180 and West = 270.

    solpos : pandas.DataFrame
        Data structure that contains solar zenith and solar azimuth.

    axis_tilt : float
        Tilt of the axis of rotation with respect to horizontal (e.g. a value of 
        0º indicates that the support axis of the photovoltaic panels is horizontal)
        in [degrees].
        
    axis_azimuth : float
        Perpendicular angle to the axis of rotation by right hand rule (e.g., a 
        value of 180º indicates a rotation from east to west) in [degrees].
        
    max_angle : float
        Maximum angle of rotation of the tracker from its horizontal position (e.g., a 
        value of 90º allows the tracker to rotate to and from a vertical position where 
        the panel faces the horizon) in [degrees].
        
    racking model : string, optional
        Racking of the PV modules. Valid strings are 'open_rack', 'close_mount', 
        and 'insulated_back'. Used to identify a parameter set for the SAPM cell 
        temperature model.
        Default = open_rack

    Returns
    -------
    mount : class
        PVlib Mount defined class.
        
    tracker : pandas.DataFrame
        Data structure that contains the following parameters:
            1. tracker_theta - Rotation angle of the tracker (zero is horizontal, and 
                               positive rotation angles are clockwise) in [degrees].
            2. aoi - Angle-of-incidence of DNI onto the rotated panel surface 
                     in [degrees].
            3. surface_tilt - Angle between the panel surface and the earth 
                              surface, accounting for panel rotation in [degrees].
            4. surface_azimuth - Azimuth of the rotated panel, determined by 
                                 projecting the vector normal to the panel’s surface 
                                 to the earth’s surface, in [degrees].

    Notes
    -----
    More details at: 
    https://pvlib-python.readthedocs.io/en/v0.9.0/generated/pvlib.pvsystem.FixedMount.html
    https://pvlib-python.readthedocs.io/en/latest/generated/pvlib.pvsystem.SingleAxisTrackerMount.html
    https://pvlib-python.readthedocs.io/en/latest/generated/pvlib.pvsystem.SingleAxisTrackerMount.get_orientation.html
    '''
    # Mount and Tracker
    if with_tracker == False:
        
        mount = pvlib.pvsystem.FixedMount(surface_tilt=surface_tilt, 
                                          surface_azimuth=surface_azimuth, 
                                          racking_model=racking_model)
        tracker = None
        
    if with_tracker == True:
        mount = pvlib.pvsystem.SingleAxisTrackerMount(axis_tilt=axis_tilt, 
                                                      axis_azimuth=axis_azimuth, 
                                                      max_angle=max_angle, 
                                                      backtrack=True, 
                                                      gcr=0.2857142857142857, 
                                                      cross_axis_tilt=0.0, 
                                                      racking_model=racking_model) 
        
        tracker = mount.get_orientation(solar_zenith=solpos.apparent_zenith, 
                                        solar_azimuth=solpos.azimuth)
        
        tracker = tracker.fillna(0)
    
    return mount, tracker