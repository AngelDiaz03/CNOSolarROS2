from IPython import get_ipython
#get_ipython().run_line_magic('matplotlib', 'inline')
import matplotlib.pyplot as plt
import numpy as np

def get_cen(ac, perc, color='#1580E4', mag='W', dwnld=False):
    '''
    Calculate the CEN (Capacidad Efectiva Neta, in spanish). According to
    CREG-081 of 2000, it is the maximum net power capacity (expressed as 
    an integer value in MW) that a plant and/or generation unit can supply 
    under normal operating conditions, measured at the commercial frontier. 
    It is calculated as the Nominal Capacity minus the Own Consumption of 
    the plant and/or generation unit.

    Parameters
    ----------
    ac : numeric
        AC power in [W].

    perc : float, optional
        Percentile value with which the minimum energy will be estimated 
        in [%]. By default, the 1st percentile is used to filter out 
        possible outliers.
        Default = 99.0

    color : string, optional
        Color of the curve generated in the CEN plot (i.e., Percentile 
        vs. AC Power).
        Default = #1580E4
        
    mag : string, optional
        To make the CEN plot easier to analyze, select the desired AC
        power magnitude to display.
        Default = W
        
    dwnld : bool, optional
        To download the displayed CEN plot.
        Default = False

    Returns
    -------
    cen_per : float 
        CEN as percentile of AC power in [MW].
        
    cen_pmax : float
        CEN as maximum AC power in [MW].

    Notes
    -----
    The calculation procedure is:
        1. Sort the AC power in ascending order.
        2. Calculate the proportional values of the sorted AC power.
        3. Calculate the CEN by the percentile of the sorted AC power.
        4. Calculate the CEN as the maximum AC power of the samples sorted.
        
    References
    ----------
    Comisión de Regulación de Energía y Gas (2000). Resolución No. 081 de 2000. 
    http://apolo.creg.gov.co/ and https://www.creg.gov.co/capacidad-efectiva-neta
    '''
    punits = {'W': 1, 'kW': 1000, 'MW': 1000000}
    
    pac = np.sort(ac)
    pac_max = np.max(ac)
    '''
    if pac_max >= 1000000:
        decimals = 0
    else:
        decimals = 4
    '''
    # Calculate the proportional values of samples
    p = 1. * np.arange(len(ac)) / (len(ac) - 1)

    # CEN
    cen_per = np.round(np.percentile(ac, perc) / 1000000, decimals=4) # MW
    cen_pmax = np.round(np.max(ac) / 1000000, decimals=4) # MW
    
    if cen_pmax >= 1:
        cen_per = np.floor(cen_per)
        cen_pmax = np.floor(cen_pmax)
    
    print(f'Pac Max. = {cen_pmax} MW\nCEN ({perc}%) = {cen_per} MW')

    # Curve plot
    plt.plot(pac/punits[mag], p, label=f'Pac Max. = {cen_pmax} MW\nCEN ({perc}%) = {cen_per} MW', linewidth=1.25, color=color)

    plt.rcParams['axes.axisbelow'] = True;

    plt.title('Capacidad Efectiva Neta', fontsize=15);
    plt.ylabel('Percentil', fontsize=13);
    plt.xlabel(f'Potencia AC, ${mag}$', fontsize=13);

    plt.tick_params(direction='out', length=5, width=0.75, grid_alpha=0.3)
    plt.xticks(rotation=0)
    plt.ylim(0, None)
    plt.xlim(None, None)
    plt.grid(True)
    plt.legend(loc='lower right', fontsize=10)
    plt.tight_layout
    
    if dwnld == True:
        plt.savefig('./downloads/cen.pdf', bbox_inches='tight')
        
    plt.show()
    
    return cen_per, cen_pmax