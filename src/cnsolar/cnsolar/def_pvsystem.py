import pvlib

def get_arrays(mount, surface_albedo, surface_type, module_type, module, mps, spi):
    '''
    Defines a pvlib.pvsystem Array class.
    
    Parameters
    ----------
    mount : class
        Mounting for the array, either on fixed-tilt racking or horizontal 
        single axis tracker. Mounting is used to determine module orientation.

    surface_albedo : float
        Ground albedo.

    surface_type : string
        Ground surface type.
        
    module_type : string
        Describes the module’s construction. Valid strings are 'glass_polymer' 
        and 'glass_glass'. Used for cell and module temperature calculations.
        
    module : dict
        Technical parameters of the PV module.
        
    mps : int
        Number of modules per string in the array.
        
    spi : int
        Number of parallel strings in the array.

    Returns
    -------
    string_array : list
        List of array defined class.

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/latest/generated/pvlib.pvsystem.Array.html
    '''
    #temp_params = pvlib.temperature.TEMPERATURE_MODEL_PARAMETERS['sapm'][module_type]
    
    string_array = [pvlib.pvsystem.Array(mount=mount, 
                                         albedo=surface_albedo, 
                                         surface_type=surface_type, 
                                         module_type=module_type, 
                                         module_parameters=module, 
                                         temperature_model_parameters=None, 
                                         modules_per_string=mps, 
                                         strings=spi)]

    return string_array

def get_pvsystem(with_tracker, tracker, string_array, surface_tilt, surface_azimuth, surface_albedo, surface_type, module_type, module, inverter, racking_model):
    '''
    Defines a pvlib.pvsystem PVSystem class. The PVSystem class defines 
    a standard set of PV system attributes and modeling functions that 
    describes the collection and interactions of PV system components. 
    
    Parameters
    ----------
    with_tracker : bool
        Parameter that checks if the mounting of the array is either on 
        fixed-tilt racking or horizontal single axis tracker.

    tracker : pandas.DataFrame
        Data structure that contains the surface tilt and azimuth values
        according to the single axis tracking rotation.

    string_array : list
        List of arrays that are part of the system.
        
    surface_tilt : float or list
        Surface tilt angles. The tilt angle is defined as degrees from 
        horizontal (e.g. surface facing up = 0, surface facing 
        horizon = 90).
        
    surface_azimuth : float or list
        Azimuth angle of the module surface. North = 0, East = 90, 
        South = 180 and West = 270.
        
    surface_albedo : float
        Ground albedo.

    surface_type : string
        Ground surface type.
        
    module_type : string
        Describes the module’s construction. Valid strings are 'glass_polymer' 
        and 'glass_glass'. Used for cell and module temperature calculations.
        
    module : dict
        Technical parameters of the PV module.
        
    inverter : dict
        Technical parameters of the inverter.
        
    racking model : string
        Racking of the PV modules. Valid strings are 'open_rack', 'close_mount', 
        and 'insulated_back'. Used to identify a parameter set for the SAPM cell 
        temperature model.

    Returns
    -------
    system : class
        PVlib PV System defined class.

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/latest/generated/pvlib.pvsystem.PVSystem.html
    '''
    #temp_params = pvlib.temperature.TEMPERATURE_MODEL_PARAMETERS['sapm'][module_type]
    
    if with_tracker == False:
        system = pvlib.pvsystem.PVSystem(arrays=string_array, 
                                         surface_tilt=surface_tilt, 
                                         surface_azimuth=surface_azimuth, 
                                         albedo=surface_albedo, 
                                         surface_type=surface_type, 
                                         module_type=module_type, 
                                         module_parameters=module, 
                                         temperature_model_parameters=None, 
                                         inverter_parameters=inverter, 
                                         racking_model=racking_model, 
                                         losses_parameters=None)

    if with_tracker == True:
        system = pvlib.pvsystem.PVSystem(arrays=string_array, 
                                         surface_tilt=tracker.surface_tilt, 
                                         surface_azimuth=tracker.surface_azimuth, 
                                         albedo=surface_albedo, 
                                         surface_type=surface_type, 
                                         module_type=module_type, 
                                         module_parameters=module, 
                                         temperature_model_parameters=None, 
                                         inverter_parameters=inverter, 
                                         racking_model=racking_model, 
                                         losses_parameters=None)
        
    return system