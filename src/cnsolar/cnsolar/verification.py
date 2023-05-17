import jsonschema

LIST_PARAMETERS = ['modules_per_string', 'strings_per_inverter']

PVWATTS_PARAMETERS = ['Pdco', 'eta_inv_nom']
SANDIA_PARAMETERS = ['Paco', 'Pdco', 'Vdco', 'Pso', 'C0', 'C1', 'C2', 'C3', 'Pnt']

MODULE_PARAMETERS = ['T_NOCT', 'Technology', 'N_s', 'I_sc_ref', 'V_oc_ref', 'I_mp_ref', 
                     'V_mp_ref', 'alpha_sc', 'beta_oc', 'gamma_r', 'STC']

SCHEMA = {'$schema': 'https://json-schema.org/draft/2020-12/schema',
          'title': 'Modelo Recurso-Potencia Solar del Acuerdo CNO XXXX',
          'description': 'Esquema JSON del Modelo Recurso-Potencia Acuerdo CNO XXXX',
          'type': 'object',

          '$defs': {'pvwatts_inverter': {'type': 'object',
                                         'properties': {'Pdco': {'type': 'number', 
                                                                 'minimum': 0.0},
                                                        'eta_inv_nom': {'type': 'number',
                                                                        'minimum': 0.0,
                                                                        'maximum': 1.0}},
                                         'required': PVWATTS_PARAMETERS,
                                         'additionalProperties': False,
                                         'minProperties': 2},
                    'sandia_inverter': {'type': 'object',
                                        'properties': {'Paco': {'type': 'number',
                                                                'minimum': 0.0},
                                                       'Pdco': {'type': 'number',
                                                                'minimum': 0.0},
                                                       'Vdco': {'type': 'number',
                                                                'minimum': 0.0},
                                                       'Pso': {'type': 'number',
                                                               'minimum': 0.0},
                                                       'C0': {'type': 'number'},
                                                       'C1': {'type': 'number'},
                                                       'C2': {'type': 'number'},
                                                       'C3': {'type': 'number'},
                                                       'Pnt': {'type': 'number',
                                                               'minimum': 0.0}},
                                        'required': SANDIA_PARAMETERS, 
                                        'additionalProperties': False,
                                        'minProperties': 9},
                    'float_array': {'type': 'array',
                                    'items': {'type': 'number',
                                              'minimum': 0.0,
                                              'maximum': 360.0},
                                    'minItems': 1},
                    'integer_array': {'type': 'array',
                                      'items': {'type': 'integer'},
                                      'minItems': 1}},
          
          'properties': {'latitude': {'type': 'number',
                                      'minimum': -5,
                                      'maximum': 15},
                         'longitude': {'type': 'number',
                                       'minimum': -80,
                                       'maximum': -60},
                         'tz': {'const': 'America/Bogota'},
                         'altitude': {'type': 'number',
                                      'minimum': -200,
                                      'maximum': 6000},
                         'surface_albedo': {'type': 'number',
                                            'minimum': 0.0,
                                            'maximum': 1.0},
                         'inverter_name': {'type': 'string'},
                         'ac_model': {'enum': ['sandia', 'pvwatts']},
                         'inverter': {'oneOf': [{'$ref': '#/$defs/pvwatts_inverter'},
                                                {'$ref': '#/$defs/sandia_inverter'}]},
                         'module_name': {'type': 'string'},
                         'module': {'type': 'object',
                                   'properties': {'T_NOCT': {'type': 'number'},
                                                  'Technology': {'type': 'string',
                                                                 'enum': ['monosi', 'multisi', 'cigs', 'cdte', 'amorphous']},
                                                  'N_s': {'type': 'integer'},
                                                  'I_sc_ref': {'type': 'number'},
                                                  'V_oc_ref': {'type': 'number'},
                                                  'I_mp_ref': {'type': 'number'},
                                                  'V_mp_ref': {'type': 'number'},
                                                  'alpha_sc': {'type': 'number'},
                                                  'beta_oc': {'type': 'number'},
                                                  'gamma_r': {'type': 'number'},
                                                  'STC': {'type': 'number'}},
                                   'required': MODULE_PARAMETERS,
                                   'additionalProperties': False,
                                   'minProperties': 11},
                         'bifacial': {'type': 'boolean'},
                         'bifaciality': {'oneOf': [{'type': 'null'},
                                                   {'type': 'number',
                                                    'minimum': 0.0,
                                                    'maximum': 1.0}]},
                         'row_height': {'oneOf': [{'type': 'null'},
                                                  {'type': 'number',
                                                   'minimum': 0.0}]},
                         'row_width': {'oneOf': [{'type': 'null'},
                                                 {'type': 'number',
                                                  'minimum': 0.0}]},
                         'num_arrays': {'type': 'integer',
                                        'minimum': 1},
                         'modules_per_string': {'$ref': '#/$defs/integer_array'},
                         'strings_per_inverter': {'$ref': '#/$defs/integer_array'},
                         'num_inverter': {'type': 'integer',
                                          'minimum': 1},
                         'with_tracker': { 'type': 'boolean'},
                         'surface_azimuth': {'oneOf': [{'type': 'null'},
                                                       {'$ref': '#/$defs/float_array'}]},
                         'surface_tilt': {'oneOf': [{'type': 'null'},
                                                    {'$ref': '#/$defs/float_array'}]},
                         'axis_tilt': {'oneOf': [{'type': 'null'},
                                                 {'$ref': '#/$defs/float_array'}]},
                         'axis_azimuth': {'oneOf': [{'type': 'null'},
                                                    {'$ref': '#/$defs/float_array'}]},
                         'max_angle': {'oneOf': [{'type': 'null'},
                                                 {'$ref': '#/$defs/float_array'}]},
                         'module_type': {'enum': ['glass_glass',
                                                  'glass_polymer']},
                         'loss': {'type': 'number',
                                  'minimum': 0.0,
                                  'maximum': 100.0},
                         'kpc': {'type': 'number',
                                 'minimum': 0.0,
                                 'maximum': 100.0},
                         'kt': {'type': 'number',
                                'minimum': 0.0,
                                'maximum': 100.0},
                         'kin': {'type': 'number',
                                 'minimum': 0.0,
                                 'maximum': 100.0},
                         'name': {'type': 'string'}},
          
          'allOf': [{'if': {'properties': {'ac_model': {'const': 'pvwatts'}},
                            'required': ['ac_model']},
                     'then': {'properties': {'inverter': {'$ref': '#/$defs/pvwatts_inverter'}}},
                     'else': {'properties': {'inverter': {'$ref': '#/$defs/sandia_inverter'}}}},
                    
                    {'if': {'properties': {'with_tracker': {'const': True}},
                            'required': ['with_tracker']},
                     'then': {'properties': {'surface_azimuth': { 'type': 'null'},
                                             'surface_tilt': { 'type': 'null'},
                                             'axis_tilt': {'$ref': '#/$defs/float_array'},
                                             'axis_azimuth': {'$ref': '#/$defs/float_array'},
                                             'max_angle': {'$ref': '#/$defs/float_array'}}},

                     'else': {'properties': {'surface_azimuth': {'$ref': '#/$defs/float_array' },
                                                                 'surface_tilt': {'$ref': '#/$defs/float_array'},
                                                                 'axis_tilt': {'type': 'null'},
                                                                 'axis_azimuth': {'type': 'null'},
                                                                 'max_angle': {'type': 'null'}}}},
                    {'if': {'properties': {'bifacial': {'const': True}},
                            'required': ['bifacial']},
                     'then': {'properties': {'bifaciality': {'type': 'number',
                                                             'minimum': 0.0,
                                                             'maximum': 1.0 },
                              'row_height': {'type': 'number',
                                             'minimum': 0.0},
                              'row_width': {'type': 'number',
                                            'minimum': 0.0}}},
                     'else': {'properties': {'bifaciality': { 'type': 'null'},
                                             'row_height': { 'type': 'null'},
                                             'row_width': { 'type': 'null'}}}}],
          'additionalProperties': False,
          'minProperties': 30}

def schema():
    return SCHEMA

# =============================================================================
# JSON verification according to Acuerdo CNO XXXX
# =============================================================================
def check_json(json_to_check):
    '''    
    It allows to validate that a given JSON follows what is stipulated in 
    the Acuerdo CNO XXXX.

    The code raises an error of type ValueError in cases where the JSON 
    file does not follow what is stipulated in the Acuerdo CNO XXXX.

    In cases where the file passes validation, an error is simply 
    not raised.
    
    Parameters
    ----------
    json_to_check : dict
        System configuration JSON to check accordance with 
        Acuerdo CNO XXXX.

    Returns
    -------
    tag : string 
        'Success' of 'Fail' according to the checking result.

    Notes
    -----
    Only when the returned tag is 'success', cnosolar will allow 
    the system configuration JSON to be downloaded.
        
    References
    ----------
    Acuerdo CNO XXXX (https://git.cno.org.co/cno/cno_solar/-/tree/main/Protocolos).
    '''
    # STEP 1. Validate if the JSON file complies with the established schema
    try: 
        jsonschema.validate(instance=json_to_check, schema=SCHEMA)
    except:
        tag = 'fail'
        jsonschema.validate(instance=json_to_check, schema=SCHEMA)
    
    # STEP 2. Validate the length of the lists (it is not possible with the jsonschema)
    if json_to_check['with_tracker'] == False:
        LIST_PARAMETERS.extend(['surface_azimuth', 'surface_tilt'])
    
    elif json_to_check['with_tracker'] == True:
        LIST_PARAMETERS.extend(['axis_tilt', 'axis_azimuth', 'max_angle'])
        
    counter = 0
    for i in LIST_PARAMETERS:
        if len(json_to_check[i]) != json_to_check['num_arrays']:
            tag = 'fail'
            counter += 1
            raise ValueError(f"Incorrect array lenght in '{i}' parameter.")
        
        if counter == 0:
            tag = 'success'
    return tag