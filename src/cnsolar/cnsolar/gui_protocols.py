#########################
#     PROTOCOLS  GUI    #
#########################

import json
import traitlets
import numpy as np
import pandas as pd
import cnosolarV2 as cno
import ipywidgets as widgets
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog
from IPython.display import display
get_ipython().run_line_magic('matplotlib', 'inline')

def execute():
    '''
    Graphical user interface for the execution of the protocols defined 
    for the calculation of the CEN (Capacidad Efectiva Neta, in spanish) 
    and the model that relates the resource and the power from the .JSON 
    system configuration file and the historical data series.
    '''
    ###############################
    #      DOCUMENTATION TAB      #
    ###############################
    gui_layout = widgets.Layout(display='flex',
                                flex_flow='row',
                                justify_content='space-between')
    
    doc_cen = widgets.HTMLMath('''
                                <h5>Información Inicial</h5>
                                <ul>
                                  <li> <b>Configuración Sistema (.JSON):</b> Seleccione el archivo .JSON de configuración del sistema. Si la planta fotovoltaica se compone de múltiples archivos .JSON de configuración, selecciónelos de forma simultánea (estos se cargarán en orden alfabético).</li>
                                  <li> <b>Serie Histórica de Datos (.CSV):</b> Seleccione el archivo .CSV de la serie histórica de datos meteorológicos. La estructura de datos sigue lo establecido por la CREG 060 de 2019, es decir, estampa temporal, $~GHI~$ y $~T_{amb}$.</li>
                                  <li> <b>Cargar Archivos:</b> Una vez seleccionados los archivos requeridos, dé clic en 'Cargar Archivos'. El ícono y la descripción del botón cambiarán para notificar la carga de los archivos.</li>
                                </ul>

                                <h5>Capacidad Efectiva Neta</h5>
                                <ul>
                                  <li> <b>Percentil [%]:</b> Valor del percentil con el que se estimará la CEN. Por defecto se tiene el percentil 99 para filtrar posibles datos atípicos.</li>
                                  <li> <b>Gráfica - Color:</b> Color de la curva generada en el gráfico de la CEN (Percentil vs. $P_{AC}$). <span style='color:red'>No se requiere para el cálculo de la CEN</span>.</li>
                                  <li> <b>Gráfica - Magnitud $P_{AC}$:</b> Para facilitar la visualización del gráfico, seleccione la magnitud en que desea presentar la Potencia AC. <span style='color:red'>No se requiere para el cálculo de la CEN</span>.</li>
                                  <li> <b>Gráfica - Descargar:</b> Seleccione la opción 'Sí' para descargar el gráfico de la CEN (se alojará en la carpeta <i>cno_solar/downloads/<span style='color:blue'>cen.pdf</span></i>). <span style='color:red'>No se requiere para el cálculo de la CEN</span>.</li>
                                  <li> <b>Calcular CEN:</b> Dé clic en el botón 'Calcular CEN' para ejecutar el algoritmo que estimará la CEN según los archivos de configuración del sistema y serie histórica de datos, además del percentil indicado.</li>
                                  <li> <b>Descargar Producción:</b> Al dar clic en este botón, se descargará un archivo .CSV con datos meteorológicos y de producción (en las estampas de tiempo de la serie histórica de datos) empleados para calcular la CEN. Se generarán archivos .CSV según la arquitectura de la planta fotovoltaica (i.e., por inversores y para la planta completa) y se alojarán en la carpeta <i>cno_solar/downloads/<span style='color:blue'>pipeline_xxx.csv</span></i>. Si la planta fotovoltaica se compone de múltiples archivos .JSON de configuración, el orden de descarga es según el orden alfabético de carga de los archivos de configuración. El ícono y la descripción del botón cambiarán para notificar la descarga.</li>
                                </ul>

                                <h5>Energía Mínima Diaria</h5>
                                Sección opcional que permite al usuario estimar la energía mínima diaria (unidades de kWh/día).
                                <ul>
                                  <li> <b>Percentil [%]:</b> Valor del percentil con el que se estimará la energía mínima. Por defecto se tiene el percentil 1 para filtrar posibles datos atípicos.</li>
                                  <li> <b>Gráfica - Color:</b> Color de la curva del comportamiento de la energía a lo largo del periodo de tiempo de la serie histórica de datos.
                                  <li> <b>Gráfica - Magnitud Energía:</b> Para facilitar la visualización del gráfico, seleccione la magnitud en que desea presentar la Energía.
                                  <li> <b>Gráfica - Descargar:</b> Seleccione la opción 'Sí' para descargar el gráfico del comportamiento de la energía a lo largo del periodo de tiempo de la serie histórica de datos (se alojará en la carpeta <i>cno_solar/downloads/<span style='color:blue'>daily_energy.pdf</span></i>).</li>
                                  <li> <b>Calcular Energía Mínima Diaria:</b> Al dar clic en este botón se ejecuta el algoritmo que estimará la energía mínima diaria según percentil indicado.</li>
                                  <li> <b>Graficar Energía Diaria:</b> Dé clic en este botón para visualizar la curva del comportamiento de la energía a lo largo del periodo de tiempo de la serie histórica de datos.</li>
                                </ul>
                                ''', layout=widgets.Layout(height='auto'))

    doc_rp = widgets.HTMLMath('''
                              <h5>Información Inicial</h5>
                              <ul>
                                <li> <b>Configuración Sistema (.JSON):</b> Seleccione el archivo .JSON de configuración del sistema. Si la planta fotovoltaica se compone de múltiples archivos .JSON de configuración, selecciónelos de forma simultánea (estos se cargarán en orden alfabético).</li>
                                <li> <b>Serie Histórica de Datos (.CSV):</b> Seleccione el archivo .CSV de la serie histórica de datos meteorológicos. La estructura de datos sigue lo establecido por la CREG 060 de 2019, es decir, estampa temporal, $~GHI~$ y $~T_{amb}$; si se añaden los parámetros $~POA~$ y $~T_{mod}$, estos priman para los cálculos de los algoritmos (e.g., no se utilizan modelos de descomposición y transposición para determinar $~POA~$ ni modelos de temperatura para determinar $~T_{mod}$). Si el instrumento de medición de la irradiancia en el plano del arreglo es una celda de referencia, nombre el encabezado de dicha columna $~Effective\_Irradiance~$ en lugar de $~POA$.</li>
                                <li> <b>Cargar Archivos:</b> Una vez seleccionados los archivos requeridos, dé clic en 'Cargar Archivos'. El ícono y la descripción del botón cambiarán para notificar la carga de los archivos.</li>
                              </ul>

                                <h5>Recurso-Potencia</h5>
                                <ul>
                                  <li> <b>Disponibilidad [%]:</b> Valor porcentual en escala entre 0 y 1 de la disponibilidad por conjunto inversores con configuración eléctrica exactamente igual (i.e., por archivo .JSON de configuración del sistema). Para múltiples inversores (i.e., múltiples archivos .JSON de configuración), separe los valores con una coma de manera ordenada.</li>
                                  <li> <b>Ejecutar:</b> Dé clic en este botón para ejecutar el algoritmo que estimará la producción de la planta fotovoltaica según el recurso indicado en los archivos de configuración del sistema y serie histórica de datos. El ícono y la descripción del botón cambiarán para notificar la ejecución del algoritmo.</li>
                                  <li> <b>Descargar Producción:</b> Al dar clic en este botón, se descargará un archivo .CSV con datos meteorológicos y de producción (en las estampas de tiempo de la serie histórica de datos) estimados por el algoritmo. Se generarán archivos .CSV según la arquitectura de la planta fotovoltaica (i.e., por inversores y para la planta completa) y se alojarán en la carpeta <i>cno_solar/downloads/<span style='color:blue'>rp_xxx.csv</span></i>. Si la planta fotovoltaica se compone de múltiples archivos .JSON de configuración, el orden de descarga es según el orden alfabético de carga de los archivos de configuración. El ícono y la descripción del botón cambiarán para notificar la descarga.</li>
                                </ul>

                                <h5>Gráfica</h5>
                                Sección opcional que permite al usuario visualizar diferentes relaciones entre parámetros. <span style='color:red'>No se requiere para el cálculo de la producción a partir del recurso</span>.
                                <ul>
                                  <li> <b>Relación:</b> Gráfica a visualizar.</li>
                                  <li> <b>Magnitud:</b> Para facilitar la visualización del gráfico, seleccione la magnitud en que desea presentar la Potencia o la Energía.</li>
                                  <li> <b>Fecha Inicial:</b> Periodo inicial a partir del cual se desea generar la gráfica según la relación seleccionada. El algoritmo inicialmente toma en cuenta el periodo de tiempo inicial de la serie histórica de datos. </li>
                                  <li> <b>Fecha Final:</b> Periodo final hasta donde se desea generar la gráfica según la relación seleccionada. El algoritmo inicialmente toma en cuenta el periodo de tiempo final de la serie histórica de datos. </li>
                                  <li> <b>Color:</b> Color de la curva de la gráfica según la relación seleccionada.</li>
                                  <li> <b>Descargar:</b> Seleccione la opción 'Sí' para descargar la gráfica según la relación seleccionada (se alojará en la carpeta <i>cno_solar/downloads/<span style='color:blue'>rp_xxx.pdf</span></i>).</li>
                                  <li> <b>Graficar:</b> Dé clic en este botón para visualizar la relación seleccionada. El usuario puede cambiar los parámetros de la relación a graficar y volver a dar clic sobre este botón para refrescar la visualización. El ícono y la descripción del botón cambiarán para notificar la ejecución del algoritmo.</li>
                                </ul>
                            ''', layout=widgets.Layout(height='auto'))


    ac_documentation = widgets.Accordion(children=[doc_cen, doc_rp])
    ac_documentation.set_title(0, 'Tab CEN')
    ac_documentation.set_title(1, 'Tab Recurso-Potencia')

    tab_doc = widgets.Box([widgets.HTML('<h4>Documentación</h4>', layout=widgets.Layout(height='auto')), 
                           widgets.VBox([widgets.Box([ac_documentation], layout=gui_layout)])], 
                           layout=widgets.Layout(display='flex',
                                                 flex_flow='column',
                                                 border='solid 0px',
                                                 align_items='stretch',
                                                 width='100%'))
    
    ###############################
    #           UPLOAD            #
    ###############################
    gui_layout = widgets.Layout(display='flex', flex_flow='row', justify_content='space-between')

    ## GUI adapted from https://codereview.stackexchange.com/questions/162920/file-selection-button-for-jupyter-notebook
    class SelectFilesButton(widgets.Button):
        '''A file widget that leverages tkinter.filedialog'''
        def __init__(self, file_to_open):
            super(SelectFilesButton, self).__init__()

            # Add the selected_files trait
            self.add_traits(files=traitlets.traitlets.Any()) # List()

            # Create the button
            self.description = 'Seleccionar'
            self.icon = 'square-o'
            #self.style.button_color = 'orange'

            # Set on click behavior
            self.on_click(self.select_files)

            self.file_to_open = file_to_open

        @staticmethod
        def select_files(b):
            '''Generate instance of tkinter.filedialog '''
            # Create Tk root
            root = Tk()

            # Hide the main window
            root.withdraw()

            # Raise the root to the top of all windows
            root.call('wm', 'attributes', '.', '-topmost', True)

            # List of selected fileswill be set to b.value
            if b.file_to_open == 'JSON':
                b.files = filedialog.askopenfilename(filetypes=(('JSON Files', '.json'),), 
                                                     multiple=True,
                                                     title='Select JSON Data File')

            elif b.file_to_open == 'CSV':
                b.files = filedialog.askopenfilename(filetypes=(('CSV Files', '.csv'),), 
                                                     multiple=False,
                                                     title='Select CSV Data File')

            b.description = 'Seleccionado'
            b.icon = 'check-square-o'
            #b.style.button_color = 'lightgreen'

    upload_config = SelectFilesButton(file_to_open='JSON')
    upload_data = SelectFilesButton(file_to_open='CSV')

    # BUTTONS
    btn = widgets.Button(value=False,
                         description='Cargar Archivos',
                         disabled=False,
                         button_style='',
                         tooltip='Cargar los archivos JSON y CSV',
                         icon='upload',
                         layout=widgets.Layout(width='100%', height='auto'))

    btn.add_traits(files=traitlets.traitlets.Dict())

    output = widgets.Output()

    def on_button_clicked(obj):
        with output:
            btn.description = 'Cargando Archivos'
            btn.icon = 'hourglass-2'
            
            output.clear_output()

            system_config = []
            for i in range(len(upload_config.files)):
                with open(upload_config.files[i]) as f:
                    system_config.append(json.load(f))

            df = cno.data.load_csv(file_name=upload_data.files, tz=system_config[0]['tz'])

            btn.files = {'system_configuration': system_config, 'df': df}
            
            btn.description = 'Archivos Cargados'
            btn.icon = 'check'
            
    btn.on_click(on_button_clicked)
    
    ###############################
    #             CEN             #
    ###############################
    # CEN Button
    w_cen = widgets.Button(value=False,
                           description='Calcular CEN',
                           disabled=False,
                           button_style='',
                           tooltip='Cálculo de la Capacidad Efectiva Neta',
                           icon='bolt',
                           style={'description_width': 'initial'},
                           layout=widgets.Layout(width='50%', height='auto'))

    w_cen.add_traits(files=traitlets.traitlets.Dict())
    cen_output = widgets.Output()

    # Percentil
    w_cenpercentil = widgets.BoundedFloatText(value=99.9, 
                                              min=0, 
                                              max=100, 
                                              step=0.1,
                                              description='',
                                              disabled=False,
                                              style={'description_width': 'initial'})
    # Plot Color
    w_cencolor = widgets.ColorPicker(concise=False,
                                     description='',
                                     value='#1580E4',
                                     disabled=False)
    # Pac Units
    w_cenunits = widgets.Dropdown(options=['W', 'kW', 'MW'], value='W', description='', style={'description_width': 'initial'})

    # Download Production Button
    download_cen = widgets.Button(value=False,
                                  description='Descargar Producción',
                                  disabled=True,
                                  button_style='',
                                  tooltip='Descarga CSV de la Producción del Sistema',
                                  icon='download',
                                  layout=widgets.Layout(width='50%', height='auto'))

    downloadcen_output = widgets.Output()

    # Plot Download
    w_downloadcenplot = widgets.Dropdown(options=[('Sí', True), ('No', False)], value=False, description='', style={'description_width': 'initial'})

    # Functions
    def on_button_clicked_cen(obj):
        with cen_output:      
            w_cen.description = 'Calculando CEN'
            w_cen.icon = 'hourglass-2'
            
            cen_output.clear_output()

            bus_pipeline = cno.pipeline.run(system_configuration=btn.files['system_configuration'], 
                                            data=btn.files['df'],
                                            availability=None,
                                            energy_units='Wh')
            
            
            ac_for_cen = bus_pipeline['plant']['ac']

            cen_per, cen_pmax = cno.cen.get_cen(ac=ac_for_cen, 
                                                perc=w_cenpercentil.value, # Percentil CEN
                                                color=w_cencolor.value,
                                                mag=w_cenunits.value,
                                                dwnld=w_downloadcenplot.value)

            w_cen.files = {'bus_pipeline':bus_pipeline, 'cen_per':cen_per, 'cen_pmax':cen_pmax}

            w_cen.description = 'CEN Calculada'
            w_cen.icon = 'check'
            
            download_cen.disabled = False
            
    w_cen.on_click(on_button_clicked_cen)

    def on_downloadcen_clicked(obj):
        with downloadcen_output:

            download_cen.description = 'Descargando Producción'
            download_cen.icon = 'hourglass-2'
            
            for sk in w_cen.files['bus_pipeline'].keys():
                main_cols = ['Zenith, degree', 'Elevation, degree', 'Azimuth, degree', 'Airmass Relative, ad',  'Airmass Absolute, ad', 'Extraterrestrial Radiation, W/m2', 'POA, W/m2', 'Tmod, C', 'Pdc, W', 'Pac, W', 'Daily Energy, Wh', 'Weekly Energy, Wh', 'Monthly Energy, Wh']

                cols_to_download = main_cols.copy()
                if w_cen.files['bus_pipeline'][sk]['bifacial'] == True:
                    cols_to_download[6:6] = ['Front Incident Irradiance, W/m2', 'Back Incident Irradiance, W/m2', 'Front Absorbed Irradiance, W/m2', 'Back Absorbed Irradiance, W/m2']

                    w_cen.files['bus_pipeline'][sk]['total_incident_front'].round(2)
                    w_cen.files['bus_pipeline'][sk]['total_incident_back'].round(2)
                    w_cen.files['bus_pipeline'][sk]['total_absorbed_front'].round(2)
                    w_cen.files['bus_pipeline'][sk]['total_absorbed_back'].round(2)

                # Check lengths
                if w_cen.files['bus_pipeline'][sk]['energy']['week'].index[-1] > btn.files['df'].index[-1]:
                    cen_energy_week = w_cen.files['bus_pipeline'][sk]['energy']['week'][:-1].round(2)
                else:
                    cen_energy_week = w_cen.files['bus_pipeline'][sk]['energy']['week'].round(2)

                if w_cen.files['bus_pipeline'][sk]['energy']['month'].index[-1] > btn.files['df'].index[-1]:
                    cen_energy_month = w_cen.files['bus_pipeline'][sk]['energy']['month'][:-1].round(2)
                else:
                    cen_energy_month = w_cen.files['bus_pipeline'][sk]['energy']['month'].round(2)

                cen_to_download = pd.concat([w_cen.files['bus_pipeline'][sk]['solpos'][['zenith', 'elevation', 'azimuth']].round(2), 
                                             w_cen.files['bus_pipeline'][sk]['airmass'].round(2),
                                             w_cen.files['bus_pipeline'][sk]['etr_nrel'].round(2),
                                             w_cen.files['bus_pipeline'][sk]['total_incident_front'],
                                             w_cen.files['bus_pipeline'][sk]['total_incident_back'],
                                             w_cen.files['bus_pipeline'][sk]['total_absorbed_front'],
                                             w_cen.files['bus_pipeline'][sk]['total_absorbed_back'],
                                             w_cen.files['bus_pipeline'][sk]['poa'].round(2),
                                             w_cen.files['bus_pipeline'][sk]['temp_cell'].round(2),
                                             w_cen.files['bus_pipeline'][sk]['p_dc'].round(2),
                                             w_cen.files['bus_pipeline'][sk]['ac'].round(2),
                                             w_cen.files['bus_pipeline'][sk]['energy']['day'].round(2),
                                             cen_energy_week,
                                             cen_energy_month], axis=1)

                cen_to_download.columns = cols_to_download
                cen_to_download.to_csv(f'./downloads/pipeline_{sk}.csv')

            download_cen.description = 'Producción Descargada'
            download_cen.icon = 'check'

    download_cen.on_click(on_downloadcen_clicked)

    ###############################
    #       ENERGÍA MÍNIMA        #
    ###############################
    eunits = {'Wh': 1, 'kWh': 1000, 'MWh': 1000000}

    # energiamin Button
    w_energiamin = widgets.Button(value=False,
                              description='Calcular Energía Mínima Diaria',
                              disabled=False,
                              tooltip='Cálculo de la Energía Mínima Diaria',
                              icon='plug',
                              style={'description_width': 'initial'},
                              layout=widgets.Layout(width='50%', height='auto'))

    w_energiamin.add_traits(files=traitlets.traitlets.Dict()) # List()
    emin_output = widgets.Output()

    # Percentil
    w_eminpercentil = widgets.BoundedFloatText(value=1, 
                                               min=0, 
                                               max=100, 
                                               step=0.1,
                                               description='',
                                               disabled=False,
                                               style={'description_width': 'initial'})
    # Plot Color
    w_emincolor = widgets.ColorPicker(concise=False,
                                      description='',
                                      value='#1580E4',
                                      disabled=False)
    # Energy Units
    w_eminunits = widgets.Dropdown(options=['Wh', 'kWh', 'MWh'], value='Wh', description='', style={'description_width': 'initial'})

    # Plot Download
    w_downloademinplot = widgets.Dropdown(options=[('Sí', True), ('No', False)], value=False, description='', style={'description_width': 'initial'})

    # energiamin Graph Button
    energiamin_graph = widgets.Button(value=False,
                                      description='Graficar Energía Diaria',
                                      disabled=False,
                                      tooltip='Gráfica de la Energía Diaria',
                                      icon='area-chart',
                                      style={'description_width': 'initial'},
                                      layout=widgets.Layout(width='50%', height='auto'))

    eminplot_output = widgets.Output()

    # Widget
    w_emin = widgets.VBox([widgets.Box([widgets.HTML('<h4>Energía Mínima Diaria</h4>', layout=widgets.Layout(height='auto'))]),
                           widgets.Box([widgets.Label('Percentil [%]'), w_eminpercentil], layout=gui_layout),
                           widgets.Box([widgets.Label('Gráfica - Color'), w_emincolor], layout=gui_layout),
                           widgets.Box([widgets.Label('Gráfica - Magnitud Energía'), w_eminunits], layout=gui_layout),
                           widgets.Box([widgets.Label('Gráfica - Descargar'), w_downloademinplot], layout=gui_layout),
                           widgets.Box([widgets.HTML('<h4> </h4>', layout=widgets.Layout(height='auto'))]),
                           widgets.Box([w_energiamin, energiamin_graph], layout=gui_layout),
                           widgets.Box([emin_output], layout=gui_layout),
                           widgets.Box([eminplot_output], layout=gui_layout)])

    ac_emin = widgets.Accordion(children=[w_emin])
    ac_emin.set_title(0, 'Energía Mínima Diaria')

    ac_energiamin = widgets.Box([ac_emin], layout=widgets.Layout(display='flex',
                                                                 flex_flow='column',
                                                                 border='solid 0px',
                                                                 align_items='stretch',
                                                                 width='100%'))

    # Functions
    def on_emin_clicked(obj):
        with emin_output:
            emin_output.clear_output()
            if len(w_cen.files['bus_pipeline'].keys()) == 1:
                energy_data = w_cen.files['bus_pipeline']['plant']['energy'] ### ¡¡¡!!!
            else:
                energy_data = w_cen.files['bus_pipeline']['plant']['energy']

            # Energía Firme: PVlib + Percentil
            e_min = cno.energia_minima.pvlib_percentile(energy=energy_data,
                                                        percentile=w_eminpercentil.value)

            w_energiamin.files = {'energiamin': e_min}

    w_energiamin.on_click(on_emin_clicked)

    def on_eminplot_clicked(obj):
        with eminplot_output:
            eminplot_output.clear_output()

            if len(w_cen.files['bus_pipeline'].keys()) == 1:
                energy_to_plot = w_cen.files['bus_pipeline']['plant']['energy']['day'] ### ¡¡¡!!!
            else:
                energy_to_plot = w_cen.files['bus_pipeline']['plant']['energy']['day']

            #Bar Plots            
            hor, ver = 13, 5
            plt.figure(figsize=(hor, ver))

            plot_label = 'Energía Mínima ({}%) = {} kWh/día'.format(w_eminpercentil.value, w_energiamin.files['energiamin'])

            dd = energy_to_plot / eunits[w_eminunits.value]
            
            plt.plot(dd, label=plot_label, marker='.', ms=6.5, linewidth=0.5, color=w_emincolor.value)

            plt.rcParams['axes.axisbelow'] = True;

            plt.title('Energía Diaria', fontsize=15);
            plt.ylabel(f'Energía, ${w_eminunits.value}$', fontsize=13);
            plt.xlabel('Tiempo', fontsize=13);

            plt.tick_params(direction='out', length=5, width=0.75, grid_alpha=0.3)
            plt.xticks(rotation=0)
            plt.ylim(0, None)
            plt.xlim(None, None)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
            plt.tight_layout                     

            if w_downloademinplot.value == True:
                plt.savefig('./downloads/daily_energy.pdf', bbox_inches='tight')

            plt.show()

    energiamin_graph.on_click(on_eminplot_clicked)

    # Tab
    widget_protocols = [widgets.Box([widgets.HTML('<h4>Información Inicial</h4>', layout=widgets.Layout(height='auto'))]),
                        widgets.Box([widgets.Label('Configuración Sistema (.JSON)'), upload_config], layout=gui_layout),
                        widgets.Box([widgets.Label('Serie Histórica de Datos (.CSV)'), upload_data], layout=gui_layout),
                        widgets.Box([widgets.HTML('<h4> </h4>', layout=widgets.Layout(height='auto'))]),
                        widgets.Box([btn, output], layout=gui_layout),
                        widgets.Box([widgets.HTML('<h4>Capacidad Efectiva Neta</h4>', layout=widgets.Layout(height='auto'))]),
                        widgets.Box([widgets.Label('Percentil [%]'), w_cenpercentil], layout=gui_layout),
                        widgets.Box([widgets.Label('Gráfica - Color'), w_cencolor], layout=gui_layout),
                        widgets.Box([widgets.Label('Gráfica - Magnitud $P_{AC}$'), w_cenunits], layout=gui_layout),
                        widgets.Box([widgets.Label('Gráfica - Descargar'), w_downloadcenplot], layout=gui_layout),
                        widgets.Box([widgets.HTML('<h4> </h4>', layout=widgets.Layout(height='auto'))]),
                        widgets.Box([w_cen, download_cen], layout=gui_layout),
                        widgets.Box([cen_output, downloadcen_output], layout=gui_layout),
                        widgets.Box([widgets.HTML('<h4> </h4>', layout=widgets.Layout(height='auto'))]),
                        widgets.Box([ac_energiamin], layout=gui_layout)]

    tab_protocols = widgets.Box(widget_protocols, layout=widgets.Layout(display='flex',
                                                                        flex_flow='column',
                                                                        border='solid 0px',
                                                                        align_items='stretch',
                                                                        width='55%'))

    ###############################
    #       RECURSO-POTENCIA      #
    ###############################
    punits = {'W': 1, 'kW': 1000, 'MW': 1000000}
    eunits = {'Wh': 1, 'kWh': 1000, 'MWh': 1000000}

    ###############################
    #           UPLOAD            #
    ###############################
    gui_layout = widgets.Layout(display='flex', flex_flow='row', justify_content='space-between')

    upload_config_rp = SelectFilesButton(file_to_open='JSON')
    upload_data_rp = SelectFilesButton(file_to_open='CSV')

    # BUTTONS
    btn_rp = widgets.Button(value=False,
                            description='Cargar Archivos',
                            disabled=False,
                            button_style='',
                            tooltip='Cargar los archivos JSON y CSV',
                            icon='upload',
                            layout=widgets.Layout(width='100%', height='auto'))

    btn_rp.add_traits(files=traitlets.traitlets.Dict())
    output_upload = widgets.Output()

    def on_button_clicked_rp(obj):
        with output:
            btn_rp.description = 'Cargando Archivos'
            btn_rp.icon = 'hourglass-2'
            
            output_upload.clear_output()

            system_config = []
            for i in range(len(upload_config_rp.files)):
                with open(upload_config_rp.files[i]) as f:
                    system_config.append(json.load(f))

            df = cno.data.load_csv(file_name=upload_data_rp.files, tz=system_config[0]['tz'])

            btn_rp.files = {'system_configuration': system_config, 'df': df}
            
            if len(upload_config_rp.files) == 1:
                v_avail = '1'
            else:
                v_avail = '1, ' * len(upload_config_rp.files)
                v_avail = v_avail[:-2]

            w_availability.value = v_avail

            if len(upload_config_rp.files) == 1:
                sdate = btn_rp.files['df'].index[0]
                edate = btn_rp.files['df'].index[-1]

            else:
                sdate = btn_rp.files['df'].index[0]
                edate = btn_rp.files['df'].index[-1]

            w_startdate.value = sdate
            w_enddate.value = edate
            
            btn_rp.description = 'Archivos Cargados'
            btn_rp.icon = 'check'

    btn_rp.on_click(on_button_clicked_rp)

    ###############################
    #             R-P             #
    ###############################
    # R-P Button
    w_rp = widgets.Button(value=False,
                          description='Ejecutar',
                          disabled=False,
                          button_style='',
                          tooltip='Cálculo de la Producción según Recurso',
                          icon='fire',
                          style={'description_width': 'initial'},
                          layout=widgets.Layout(width='50%', height='auto'))

    w_rp.add_traits(files=traitlets.traitlets.Dict())
    rp_output = widgets.Output()

    # Graph Button
    plot_btn = widgets.Button(value=False,
                              description='Graficar',
                              disabled=False,
                              tooltip='Gráfica de la Producción',
                              icon='area-chart',
                              style={'description_width': 'initial'},
                              layout=widgets.Layout(width='100%', height='auto'))

    plot_output = widgets.Output()

    # Download Button
    download_rp = widgets.Button(value=False,
                                 description='Descargar Producción',
                                 disabled=True,
                                 button_style='',
                                 tooltip='Descarga CSV de la Producción del Sistema',
                                 icon='download',
                                 layout=widgets.Layout(width='50%', height='auto'))

    downloadrp_output = widgets.Output()

    # Availability
    w_availability = widgets.Text(value=None, description='', style={'description_width': 'initial'})

    # Plot Color
    w_plotcolor = widgets.ColorPicker(concise=False, description='', value='#1580E4', style={'description_width': 'initial'})

    # Relation to Plot
    w_relation = widgets.Dropdown(options=['',
                                           'Tiempo - Potencia DC',
                                           'Tiempo - Potencia AC',
                                           'Tiempo - Energía Diaria',
                                           'Tiempo - Energía Semanal',
                                           'Tiempo - Energía Mensual',
                                           'Irradiancia - Potencia DC',
                                           'Irradiancia - Potencia AC'], 
                                  value='', description='', style={'description_width': 'initial'})

    # Plot Download
    w_downloadplot = widgets.Dropdown(options=[('Sí', True), ('No', False)], value=False, description='', style={'description_width': 'initial'})

    # Power/Energy Units
    w_units = widgets.Dropdown(options=[''], value='', description='', style={'description_width': 'initial'})

    # Dates
    w_startdate = widgets.DatePicker(description='')

    w_enddate = widgets.DatePicker(description='')

    # Functions
    def handle_units(change):
        if change['new'] in ['Tiempo - Potencia DC', 'Tiempo - Potencia AC', 'Irradiancia - Potencia DC', 'Irradiancia - Potencia AC']:
            units_opt = ['W', 'kW', 'MW']
            units_init = 'W'

        elif change['new'] in ['Tiempo - Energía Diaria', 'Tiempo - Energía Semanal', 'Tiempo - Energía Mensual']:
            units_opt = ['Wh', 'kWh', 'MWh']
            units_init = 'Wh'

        else:
            units_opt = ['']
            units_init = ''

        w_units.options = units_opt
        w_units.value = units_init

    w_relation.observe(handle_units, 'value')

    def str_to_list(string):
        l = []
        l.append('[')
        l.append(string) # l.append(string.value)
        l.append(']')
        return json.loads(''.join(l))

    def on_button_clicked_bus(obj):
        with rp_output:     
            w_rp.description = 'Ejecutando'
            w_rp.icon = 'hourglass-2'
            
            rp_output.clear_output()
            
            bus_pipeline = cno.pipeline.run(system_configuration=btn_rp.files['system_configuration'], 
                                            data=btn_rp.files['df'],
                                            availability=str_to_list(w_availability.value),
                                            energy_units='Wh')

            w_rp.files = {'bus_pipeline': bus_pipeline}
            
            w_rp.description = 'Ejecutado'
            w_rp.icon = 'check'
            
            download_rp.disabled = False

    w_rp.on_click(on_button_clicked_bus)

    def on_downloadrp_clicked(obj):
        with downloadrp_output:

            download_rp.description = 'Descargando'
            download_rp.icon = 'hourglass-2'
            
            for sk in w_rp.files['bus_pipeline'].keys():
                main_cols = ['Zenith, degree', 'Elevation, degree', 'Azimuth, degree', 'Airmass Relative, ad',  'Airmass Absolute, ad', 'Extraterrestrial Radiation, W/m2', 'POA, W/m2', 'Tmod, C', 'Pdc, W', 'Pac, W', 'Daily Energy, Wh', 'Weekly Energy, Wh', 'Monthly Energy, Wh']
                
                cols_to_download = main_cols.copy()
                if btn_rp.files['system_configuration'][0]['bifacial'] == True:
                    cols_to_download[6:6] = ['Front Incident Irradiance, W/m2', 'Back Incident Irradiance, W/m2', 'Front Absorbed Irradiance, W/m2', 'Back Absorbed Irradiance, W/m2']

                    w_rp.files['bus_pipeline'][sk]['total_incident_front'].round(2)
                    w_rp.files['bus_pipeline'][sk]['total_incident_back'].round(2)
                    w_rp.files['bus_pipeline'][sk]['total_absorbed_front'].round(2)
                    w_rp.files['bus_pipeline'][sk]['total_absorbed_back'].round(2)

                # Check lengths
                if w_rp.files['bus_pipeline'][sk]['energy']['week'].index[-1] > btn_rp.files['df'].index[-1]:
                    rp_energy_week = w_rp.files['bus_pipeline'][sk]['energy']['week'][:-1].round(2)
                else:
                    rp_energy_week = w_rp.files['bus_pipeline'][sk]['energy']['week'].round(2)

                if w_rp.files['bus_pipeline'][sk]['energy']['month'].index[-1] > btn_rp.files['df'].index[-1]:
                    rp_energy_month = w_rp.files['bus_pipeline'][sk]['energy']['month'][:-1].round(2)
                else:
                    rp_energy_month = w_rp.files['bus_pipeline'][sk]['energy']['month'].round(2)

                rp_to_download = pd.concat([w_rp.files['bus_pipeline'][sk]['solpos'][['zenith', 'elevation', 'azimuth']].round(2), 
                                            w_rp.files['bus_pipeline'][sk]['airmass'].round(2),
                                            w_rp.files['bus_pipeline'][sk]['etr_nrel'].round(2),
                                            w_rp.files['bus_pipeline'][sk]['total_incident_front'],
                                            w_rp.files['bus_pipeline'][sk]['total_incident_back'],
                                            w_rp.files['bus_pipeline'][sk]['total_absorbed_front'],
                                            w_rp.files['bus_pipeline'][sk]['total_absorbed_back'],
                                            w_rp.files['bus_pipeline'][sk]['poa'].round(2),
                                            w_rp.files['bus_pipeline'][sk]['temp_cell'].round(2),
                                            w_rp.files['bus_pipeline'][sk]['p_dc'].round(2),
                                            w_rp.files['bus_pipeline'][sk]['ac'].round(2),
                                            w_rp.files['bus_pipeline'][sk]['energy']['day'].round(2),
                                            rp_energy_week,
                                            rp_energy_month], axis=1)

                rp_to_download.columns = cols_to_download
                rp_to_download.to_csv(f'./downloads/rp_{sk}.csv')
                    
            download_rp.description = 'Descargado'
            download_rp.icon = 'check'

    download_rp.on_click(on_downloadrp_clicked)

    def on_plot_clicked(obj):
        with plot_output:
            plot_output.clear_output()

            df_to_plot = w_rp.files['bus_pipeline']['plant']
            
            tz_loc = df_to_plot['p_dc'].index[0].tz
            try:
                start_plot_date = pd.to_datetime(w_startdate.value, format="%Y-%m-%d %H:%M:%S").tz_localize(tz_loc)
            except:
                start_plot_date = pd.to_datetime(w_startdate.value, format="%Y-%m-%d %H:%M:%S").tz_convert(tz_loc)
                
            try:
                end_plot_date = pd.to_datetime(w_enddate.value, format="%Y-%m-%d %H:%M:%S").tz_localize(tz_loc)
            except:
                end_plot_date = pd.to_datetime(w_enddate.value, format="%Y-%m-%d %H:%M:%S").tz_convert(tz_loc)
            
            if w_relation.value == 'Tiempo - Potencia DC':
                yy = df_to_plot['p_dc'][start_plot_date:end_plot_date] / punits[w_units.value]

                title = 'Comportamiento Potencia DC'
                xlab = 'Tiempo'
                ylab = f'Potencia, ${w_units.value}$'
                rot = 30
                down_label = 'dc_power'

                plt.plot(yy, marker='.', ms=6.5, linewidth=0.5, color=w_plotcolor.value)

            elif w_relation.value == 'Tiempo - Potencia AC':
                yy = df_to_plot['ac'][start_plot_date:end_plot_date] / punits[w_units.value]

                title = 'Comportamiento Potencia AC'
                xlab = 'Tiempo'
                ylab = f'Potencia, ${w_units.value}$'
                rot = 30
                down_label = 'ac_power'

                plt.plot(yy, marker='.', ms=6.5, linewidth=0.5, color=w_plotcolor.value)

            elif w_relation.value == 'Tiempo - Energía Diaria':
                yy = df_to_plot['energy']['day'][start_plot_date:end_plot_date] / eunits[w_units.value]

                title = 'Comportamiento Energía Diaria'
                xlab = 'Tiempo'
                ylab = f'Energía, ${w_units.value}$'
                rot = 30
                down_label = 'daily_energy'

                plt.plot(yy, marker='.', ms=6.5, linewidth=0.5, color=w_plotcolor.value)

            elif w_relation.value == 'Tiempo - Energía Semanal':
                yy = df_to_plot['energy']['week'][start_plot_date:end_plot_date] / eunits[w_units.value]

                title = 'Comportamiento Energía Semanal'
                xlab = 'Tiempo'
                ylab = f'Energía, ${w_units.value}$'
                rot = 30
                down_label = 'weekly_energy'

                plt.plot(yy, marker='.', ms=6.5, linewidth=0.5, color=w_plotcolor.value)

            elif w_relation.value == 'Tiempo - Energía Mensual':
                yy = df_to_plot['energy']['month'][start_plot_date:end_plot_date] / eunits[w_units.value]

                title = 'Comportamiento Energía Mensual'
                xlab = 'Tiempo'
                ylab = f'Energía, ${w_units.value}$'
                rot = 30
                down_label = 'monthly_energy'

                plt.plot(yy, marker='.', ms=6.5, linewidth=0.5, color=w_plotcolor.value)

            elif w_relation.value == 'Irradiancia - Potencia DC':
                xx = df_to_plot['poa'][start_plot_date:end_plot_date]
                yy = df_to_plot['p_dc'][start_plot_date:end_plot_date] / punits[w_units.value]

                title = 'Relación Irradiancia vs. Potencia DC'
                xlab = 'Irradiancia POA, W/m2'
                ylab = f'Potencia, ${w_units.value}$'
                rot = 0
                down_label = 'irradiance_dcpower'

                plt.plot(xx, yy, ls='', marker='.', ms=0.5, fillstyle='none', color=w_plotcolor.value)

            else:
                xx = df_to_plot['poa'][start_plot_date:end_plot_date]
                yy = df_to_plot['ac'][start_plot_date:end_plot_date] / punits[w_units.value]

                title = 'Relación Irradiancia vs. Potencia AC'
                xlab = 'Irradiancia POA, W/m2'
                ylab = f'Potencia, ${w_units.value}$'
                rot = 0
                down_label = 'irradiance_acpower'

                plt.plot(xx, yy, ls='', marker='.', ms=0.5, fillstyle='none', color=w_plotcolor.value)

            # Plots
            plt.rcParams['axes.axisbelow'] = True;

            plt.title(title, fontsize=15);
            plt.ylabel(ylab, fontsize=13);
            plt.xlabel(xlab, fontsize=13);

            plt.tick_params(direction='out', length=5, width=0.75, grid_alpha=0.3)
            plt.xticks(rotation=rot)
            plt.ylim(0, None)
            plt.xlim(None, None)
            plt.grid(True)
            plt.tight_layout

            if w_downloadplot.value == True:
                plt.savefig(f'./downloads/rp_{down_label}.pdf', bbox_inches='tight')

            plt.show()

    plot_btn.on_click(on_plot_clicked)

    # Tab
    widget_rp = [widgets.Box([widgets.HTML('<h4>Información Inicial</h4>', layout=widgets.Layout(height='auto'))]),
                 widgets.Box([widgets.Label('Configuración Sistema (.JSON)'), upload_config_rp], layout=gui_layout),
                 widgets.Box([widgets.Label('Serie Histórica de Datos (.CSV)'), upload_data_rp], layout=gui_layout),
                 widgets.Box([widgets.HTML('<h4> </h4>', layout=widgets.Layout(height='auto'))]),
                 widgets.Box([btn_rp, output_upload], layout=gui_layout),
                 widgets.Box([widgets.HTML('<h4>Recurso-Potencia</h4>', layout=widgets.Layout(height='auto'))]),
                 widgets.Box([widgets.Label('Disponibilidad [%]'), w_availability], layout=gui_layout),
                 widgets.Box([widgets.HTML('<h4> </h4>', layout=widgets.Layout(height='auto'))]),
                 widgets.Box([w_rp, download_rp], layout=gui_layout),
                 widgets.Box([rp_output, downloadrp_output], layout=gui_layout),
                 widgets.Box([widgets.HTML('<h4>Gráfica</h4>', layout=widgets.Layout(height='auto'))]),
                 widgets.Box([widgets.Label('Relación'), w_relation], layout=gui_layout),
                 widgets.Box([widgets.Label('Magnitud'), w_units], layout=gui_layout),
                 widgets.Box([widgets.Label('Fecha Inicial'), w_startdate], layout=gui_layout),
                 widgets.Box([widgets.Label('Fecha Final'), w_enddate], layout=gui_layout),
                 widgets.Box([widgets.Label('Color'), w_plotcolor], layout=gui_layout),
                 widgets.Box([widgets.Label('Descargar'), w_downloadplot], layout=gui_layout),
                 widgets.Box([widgets.HTML('<h4> </h4>', layout=widgets.Layout(height='auto'))]),
                 widgets.Box([plot_btn], layout=gui_layout),
                 widgets.Box([plot_output], layout=gui_layout)]

    tab_rp = widgets.Box(widget_rp, layout=widgets.Layout(display='flex',
                                                          flex_flow='column',
                                                          border='solid 0px',
                                                          align_items='stretch',
                                                          width='55%'))

    ###############################
    #            GUI              #
    ###############################
    item_layout = widgets.Layout(margin='0 0 25px 0')

    tab = widgets.Tab([tab_doc, tab_protocols, tab_rp], layout=item_layout)
    tab.set_title(0, 'Documentación')
    tab.set_title(1, 'CEN')
    tab.set_title(2, 'Recurso-Potencia')
    

    dashboard = widgets.VBox([tab])
    display(dashboard)