import pvlib

def get_parameters(latitude, longitude, tz, altitude, datetime):
    '''
    Wrapper that defines a pvlib.location Location class and estimates the
    solar position parameters, airmass and extraterrestrial DNI.
    
    Parameters
    ----------
    latitude : float
        Latitude based on the location of the PV plant in decimal degrees notation.

    longitude : float
        Longitude based on the location of the PV plant in decimal degrees notation.

    tz : string
        Time zone of the location of the PV plant.
        
    altitude : float
        Altitude based on the location of the PV plant from sea level in [m].
    
    datetime : numeric
        Time stamps of the historical data series in pandas.DatetimeIndex format.

    Returns
    -------
    location : class
        PVlib Location defined class.
        
    solpos : pandas.DataFrame
        Data structure that contains solar zenith and solar azimuth.
        
    airmass : pandas.DataFrame
        Data structure that contains relative and absolute airmass.
        
    etr_nrel : numeric
        Extraterrestrial radiation from time stamps of the historical 
        data series.

    Notes
    -----
    More details at: 
    https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.location.Location.html
    https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.location.Location.get_solarposition.html
    https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.location.Location.get_airmass.html
    https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.irradiance.get_extra_radiation.html
    '''
    # Geographic Location
    location = pvlib.location.Location(latitude, longitude, tz, altitude)
    
    # Solar Position Parameters
    solpos = location.get_solarposition(times=datetime, 
                                        method='nrel_numpy')

    # Airmass
    airmass = location.get_airmass(times=datetime, 
                                   solar_position=solpos, 
                                   model='kastenyoung1989')

    # Extraterrestrial DNI
    etr_nrel = pvlib.irradiance.get_extra_radiation(datetime_or_doy=datetime, 
                                                    method='NREL', 
                                                    solar_constant=1361)

    return location, solpos, airmass, etr_nrel