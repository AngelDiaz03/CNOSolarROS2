import pvlib

def from_sandia(poa, temp_air, wind_speed, temp_params):
    '''
    Calculate the cell temperature using the Sandia Array Performance Model.

    Parameters
    ----------
    poa : numeric
        Plane-of-array irradiance in [W/m^2].

    temp_air : numeric
        Ambient temperature in [ºC].

    wind_speed : numeric
        Wind speed at a height of 10 meters in [m/s].
        
    temp_params : dict
        Temperature parameters (i.e., :math:`a`, :math:`b` and :math:`\Delta T`) 
        defined by the module construction and its mounting.

    Returns
    -------
    temp_cell : numeric 
        Cell temperature in [ºC].

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.temperature.sapm_cell.html
    '''
    temp_cell = pvlib.temperature.sapm_cell(poa_global=poa, 
                                            temp_air=temp_air, 
                                            wind_speed=wind_speed, 
                                            a=temp_params['a'], 
                                            b=temp_params['b'], 
                                            deltaT=temp_params['deltaT'], 
                                            irrad_ref=1000.0)
    
    return temp_cell

def from_tnoct(poa, temp_air, tnoct, mount_temp=0):
    '''
    Calculate the cell temperature using a simplified steady state model. A linear
    relationship between the solar irradiance and the difference between the cell 
    and the ambient temperatures is assumed.

    Parameters
    ----------
    poa : numeric
        Plane-of-array irradiance in [W/m^2].

    temp_air : numeric
        Ambient temperature in [ºC].

    tnoct : float
        Nominal operating cell temperature (NOCT) in [ºC].
        
    mount_temp : float, optional
        Installed nominal operating cell temperature (INOCT) in [ºC].
        Default = 0

    Returns
    -------
    temp_cell : numeric 
        Cell temperature in [ºC].

    References
    ----------
    A. Smets, K. Jäger, O. Isabella, R. Van Swaaij & M. Zeman (2016). Solar Energy: 
    The physics and engineering of photovoltaic conversion, technologies and systems, 
    1st Edition, pp.54. Netherlands and Germany: UIT Cambridge.
    '''
    if mount_temp != 0:
        temp_cell = temp_air + ((tnoct - 20)/800)*poa
    else:
        tinoct = tnoct - mount_temp
        temp_cell = temp_air + ((tinoct - 20)/800)*poa
    
    return temp_cell

def from_tmod(poa, temp_mod, temp_params):
    '''
    Calculate cell temperature from module temperature using the Sandia Array
    Performance Model.

    Parameters
    ----------
    poa : numeric
        Plane-of-array irradiance in [W/m^2].

    temp_mod : numeric
        Temperature of back of module surface in [ºC].
        
    temp_params : dict
        Temperature parameters (i.e., :math:`a`, :math:`b` and :math:`\Delta T`) 
        defined by the module construction and its mounting.

    Returns
    -------
    temp_cell : numeric 
        Cell temperature in [ºC].

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.temperature.sapm_cell_from_module.html
    '''
    temp_cell = pvlib.temperature.sapm_cell_from_module(module_temperature=temp_mod, 
                                                        poa_global=poa, 
                                                        deltaT=temp_params['deltaT'], 
                                                        irrad_ref=1000.0)
    
    return temp_cell

def from_pvsyst(poa, temp_air, wind_speed=1.0, u_c=29.0, u_v=0.0, eta=0.1, alpha=0.9):
    '''
    Calculate cell temperature using an empirical heat loss factor model as 
    implemented in PVsyst.

    Parameters
    ----------
    poa : numeric
        Plane-of-array irradiance in [W/m^2].

    temp_air : numeric
        Ambient temperature in [ºC].

    wind_speed : numeric, optional
        Wind speed at a height of 10 meters in [m/s].
        Default = 1.0

    u_c : float, optional
        Combined heat loss factor coefficient. The default value is 
        representative of freestanding modules with the rear surfaces 
        exposed to open air (e.g., rack mounted).
        Default = 29.0
        
    u_v : float, optional
        Combined heat loss factor influenced by wind.
        Default = 0.0
        
    eta : float, optional
        Unitless module external efficiency as a fraction of DC power and POA 
        irradiance times module area.
        Default = 0.1
        
    alpha : float, optional
        Absorption coefficient. 
        Default = 0.9

    Returns
    -------
    temp_cell : numeric 
        Cell temperature in [ºC].

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.temperature.pvsyst_cell.html
    '''
    temp_cell = pvlib.temperature.pvsyst_cell(poa_global=poa, 
                                              temp_air=temp_air, 
                                              wind_speed=wind_speed,
                                              u_c=u_c, 
                                              u_v=u_v, 
                                              eta_m=None, 
                                              module_efficiency=eta, 
                                              alpha_absorption=alpha)
    
    return temp_cell

def from_sam(poa, temp_air, tnoct, eta, wind_speed=1.0, eff_irrad=None, transmittance=0.9, height=1, standoff=4):
    '''
    Calculate cell temperature using the model from System Advisor Model.

    Parameters
    ----------
    poa : numeric
        Plane-of-array irradiance in [W/m^2].

    temp_air : numeric
        Ambient temperature in [ºC].

    tnoct : float
        Nominal operating cell temperature (NOCT) in [ºC].
        
    eta : float, optional
        Unitless module external efficiency as a fraction of DC power and POA 
        irradiance times module area.
        Default = 0.1

    wind_speed : numeric, optional
        Wind speed at a height of 10 meters in [m/s].
        Default = 1.0

    eff_irrad : float, optional
        Irradiance that is converted to photocurrent, i.e., effective irradiance.
        Default = None
        
    transmittance : float, optional
        Unitless coefficient for combined transmittance and absorptance effects.
        Default = 0.9
        
    height : int, optional
        Height of array above ground in stories (1 story is about 3 meters). Must be 
        either 1 or 2. For systems elevated less than one story, use 1. If system is 
        elevated more than two stories, use 2.
        Default = 1
        
    standoff : int, optional
        Distance between array mounting and mounting surface in [inches]. Use default 
        if system is ground-mounted.
        Default = 4

    Returns
    -------
    temp_cell : numeric 
        Cell temperature in [ºC].

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.temperature.noct_sam.html
    '''
    temp_cell = pvlib.temperature.noct_sam(poa_global=poa, 
                                           temp_air=temp_air, 
                                           wind_speed=wind_speed, 
                                           noct=tnoct, 
                                           module_efficiency=eta, 
                                           effective_irradiance=eff_irrad, 
                                           transmittance_absorptance=transmittance, 
                                           array_height=height, 
                                           mount_standoff=standoff)
    
    return temp_cell