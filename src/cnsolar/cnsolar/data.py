import pvlib
import pandas as pd

def psm3_to_df(folder_name, prefix, start_year, end_year, tz, sort_index=False):
    '''
    Read an NSRDB PSM3 weather files (formatted as SAM CSV) into pandas 
    DataFrame. Tipically, the NSRDB PSM3 weather files are stored in a 
    single folder at the time of download. The NSRDB PSM3 weather files 
    are formatted as prefix_year.csv, and there is one per year.
    
    Parameters
    ----------
    folder_name : string
        Filename of a folder containing the files to read.

    prefix : string
        String before the '_' of the NSRDB PSM3 weather files.

    start_year : int
        Start year of the NSRDB PSM3 weather files time period.
        
    end_year : int
        Final year of the NSRDB PSM3 weather files time period.
        
    tz : string
        Time zone of the location of the NSRDB PSM3 weather files.

    Returns
    -------
    df : pandas.DataFrame
        Timeseries data from NREL PSM3.

    Notes
    -----
    More details at: https://pvlib-python.readthedocs.io/en/stable/generated/pvlib.iotools.read_psm3.html
    '''
    frames = []
    for i in range(start_year, end_year+1):
        frames.append(pvlib.iotools.read_psm3(filename=f'./data/{folder_name}/{prefix}_{i}.csv')[0])

    df = pd.concat(frames)
    df = df.fillna(0)
    df = df.tz_convert(tz)
    
    if sort_index == True:
        df = df.sort_index()

    return df

def load_csv(file_name, tz):
    '''
    Read a comma-separated values (CSV) file into pandas DataFrame.
    
    Parameters
    ----------
    file_name : string
        Filename of a file containing data to read.
        
    tz : string
        Time zone of the location of the NSRDB PSM3 weather files.

    Returns
    -------
    df : pandas.DataFrame
        Timeseries data.

    Notes
    -----
    More details at: https://pandas.pydata.org/docs/reference/api/pandas.read_csv.html
    '''
    df = pd.read_csv(filepath_or_buffer=file_name, 
                     sep=',',
                     decimal='.',
                     header='infer',
                     index_col='Unnamed: 0')

    df.index = pd.DatetimeIndex(data=df.index)
    df = df.fillna(0)
    
    if df.index.tz == None:
        try:
            df = df.tz_convert(tz)
        except:
            df = df.tz_localize(tz)

    return df