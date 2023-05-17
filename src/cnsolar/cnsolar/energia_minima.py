import numpy as np
import pandas as pd
from calendar import monthrange

units = {'kWh': 1000, 'MWh': 1000000}

def pvlib_prom(energy):
    '''
    Calculate the daily minimum energy.

    Parameters
    ----------
    energy : dict
        Parameter that contains daily, weekly and monthly energy in [Wh].

    Returns
    -------
    e_min : float
        Daily minimum energy in [kWh/day].

    Notes
    -----
    The calculation procedure is:
        1. Monthly energy from PVlib simulation.
        2. Daily energy using CREG-201 of 2017 method.
        3. Daily minimum energy as the minimum value of daily energy samples.
        
    References
    ----------
    Comisión de Regulación de Energía y Gas (2017).Resolución No. 201 de 2017. 
    http://apolo.creg.gov.co/
    '''
    # Step 1: Daily energy estimate
    En = []
    for i in range(len(energy['month'])):
        e_kWh = energy['month']['energy'][i] / 1000 # kWh
        En.append(e_kWh / monthrange(energy['month'].index[i].year, energy['month'].index[i].month)[1])

    efirme = energy['month'].copy()
    efirme['En'] = En
    
    # Step 2: Energía Firme
    e_min = np.round(np.min(efirme['En']), 2) # kWh/day
    print(f'Energía Mínima = {e_min} kWh/día')
    
    return e_min

def pvlib_min(energy):
    '''
    Calculate the daily minimum energy.

    Parameters
    ----------
    energy : dict
        Parameter that contains daily, weekly and monthly energy in [Wh].

    Returns
    -------
    e_min : float
        Daily minimum energy in [kWh/day].

    Notes
    -----
    The calculation procedure is:
        1. Daily energy from PVlib simulation.
        3. Daily minimum energy as the minimum value of daily energy samples.
    '''
    e = energy['day']
    if isinstance(e, pd.Series):
        e = pd.DataFrame({'energy': e})
    
    e_min = np.round(np.min(e['energy'].loc[e['energy'] != 0]) / 1000, 2) # kWh
    print(f'Energía Mínima = {e_min} kWh/día')
    return e_min

def pvlib_percentile(energy, percentile):
    '''
    Calculate the daily minimum energy.

    Parameters
    ----------
    energy : dict
        Parameter that contains daily, weekly and monthly energy in [Wh].
        
    percentile : float
        Percentile value with which the daily minimum energy will be 
        estimated in [%].

    Returns
    -------
    e_min : float
        Daily minimum energy in [kWh/day].

    Notes
    -----
    The calculation procedure is:
        1. Daily energy from PVlib simulation.
        3. Daily minimum energy as a percentile value of daily energy samples.
    '''
    e = energy['day']
    if isinstance(e, pd.Series):
        e = pd.DataFrame({'energy': e})
    
    e_min = np.round(np.percentile(e['energy'].loc[e['energy'] != 0], percentile) / 1000, 2) # kWh
    print(f'Energía Mínima = {e_min} kWh/día')
    return e_min