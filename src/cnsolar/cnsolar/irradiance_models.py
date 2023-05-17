import pvlib
import numpy as np

# Decomposition Model: DISC
def decomposition(ghi, solpos, datetime):
    '''
    Estimate Direct Normal Irradiance from Global Horizontal Irradiance using 
    the Direct Insolation Simulation Code (DISC) model. The DISC algorithm 
    converts global horizontal irradiance to direct normal irradiance through 
    empirical relationships between the global and direct clearness indices. 
    The pvlib implementation limits the clearness index to 1.
    
    Parameters
    ----------
    ghi : numeric
        Global horizontal irradiance in [W/m2].

    solpos : pandas.DataFrame
        Data structure that contains solar zenith and solar azimuth.

    datetime : numeric
        Time stamps of the historical data series in pandas.DatetimeIndex format.

    Returns
    -------
    disc : pandas.DataFrame
        Data structure that contains the following parameters:
            1. dni - Modeled direct normal irradiance provided by the Direct 
                     Insolation Simulation Code (DISC) model in [W/m2].
            2. kt - Ratio of global to extraterrestrial irradiance on a 
                    horizontal plane.
            3. airmass - Airmass.
            4. dhi - Diffuse horizontal irradiance calculated by the fraction
                     of the difference of GHI and DNI, and the cosine of 
                     solar zenith in [W/m2].

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.irradiance.disc.html
    '''
    disc = pvlib.irradiance.disc(ghi=ghi, 
                                 solar_zenith=solpos.zenith, 
                                 datetime_or_doy=datetime, 
                                 pressure=None, # Absolute Airmass: 101325, Relative Airmass: None
                                 min_cos_zenith=0.065, # Default
                                 max_zenith=87, # Default
                                 max_airmass=12) # Default

    disc['dhi'] = ghi - disc.dni*np.cos(np.radians(solpos.zenith))

    return disc

# Transposition Model: Perez-Ineichen 1990
def transposition(with_tracker, tracker, surface_tilt, surface_azimuth, solpos, disc, ghi, etr_nrel, airmass, surface_albedo, surface_type):
    '''
    Determine total in-plane irradiance and its beam, sky diffuse and ground 
    reflected components, using the Perez-Ineichen 1990 sky diffuse irradiance 
    model.
    
    Parameters
    ----------
    with_tracker : bool
        Parameter that checks if the mounting of the array is either on 
        fixed-tilt racking or horizontal single axis tracker.

    tracker : pandas.DataFrame
        Data structure that contains the surface tilt and azimuth values
        according to the single axis tracking rotation.
    
    surface_tilt : float or list
        Surface tilt angles. The tilt angle is defined as degrees from 
        horizontal (e.g. surface facing up = 0, surface facing 
        horizon = 90).
        
    surface_azimuth : float or list
        Azimuth angle of the module surface. North = 0, East = 90, 
        South = 180 and West = 270.
   
    solpos : pandas.DataFrame
        Data structure that contains solar zenith and solar azimuth.
   
    disc : pandas.DataFrame
        Data structure that contains DNI and DHI irradiance components.
    
    ghi : numeric
        Global horizontal irradiance in [W/m2].
        
    etr_nrel : numeric
        Extraterrestrial radiation from time stamps of the historical 
        data series.

    airmass : pandas.DataFrame
        Data structure that contains relative and absolute airmass.
        
    surface_albedo : float
        Ground albedo.

    surface_type : string
        Ground surface type.

    Returns
    -------
    poa : pandas.DataFrame
        Data structure that contains the following parameters:
            1. poa_global - POA global irradiance in [W/m2].
            2. poa_direct - POA direct normal irradiance in [W/m2].
            3. poa_diffuse - POA diffuse irradiance in [W/m2].
            4. poa_sky_diffuse - POA sky diffuse irradiance in [W/m2].
            5. poa_ground_diffuse - POA ground diffuse irradiance in [W/m2].

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.irradiance.get_total_irradiance.html
    '''
    if with_tracker == False:
        poa = pvlib.irradiance.get_total_irradiance(surface_tilt=surface_tilt, 
                                                    surface_azimuth=surface_azimuth, 
                                                    solar_zenith=solpos.zenith, 
                                                    solar_azimuth=solpos.azimuth, 
                                                    dni=disc.dni, 
                                                    ghi=ghi, 
                                                    dhi=disc.dhi, 
                                                    dni_extra=etr_nrel, 
                                                    airmass=airmass.airmass_relative, 
                                                    albedo=surface_albedo, 
                                                    surface_type=surface_type, 
                                                    model='perez', 
                                                    model_perez='allsitescomposite1990')
    
    if with_tracker == True:
        poa = pvlib.irradiance.get_total_irradiance(surface_tilt=tracker.surface_tilt, 
                                                    surface_azimuth=tracker.surface_azimuth, 
                                                    solar_zenith=solpos.zenith, 
                                                    solar_azimuth=solpos.azimuth, 
                                                    dni=disc.dni, 
                                                    ghi=ghi, 
                                                    dhi=disc.dhi, 
                                                    dni_extra=etr_nrel, 
                                                    airmass=airmass.airmass_relative, 
                                                    albedo=surface_albedo, 
                                                    surface_type=surface_type, 
                                                    model='perez', 
                                                    model_perez='allsitescomposite1990')

    return poa