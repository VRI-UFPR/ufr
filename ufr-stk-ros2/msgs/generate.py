import jinja2
import json
from pathlib import Path

# Tipos 
# int   8,16,32,64 :  2,4,6,8
# uint  8,16,32,64 :  3,5,7,9
# float 32,64      :  10,11
# string           :  17


def render_message(filename: str):
    # configura a pasta de templates
    env = jinja2.Environment(loader=jinja2.FileSystemLoader('./templates'))

    # abre um arquivo json, exemplo sensor_msgs/Temperature.json
    fd = open(filename)
    raw = fd.read()
    fd.close()
    msg = json.loads(raw)

    # Pega os dados
    data_type_name = msg['type_description_msg']['type_description']['type_name']
    fields = msg['type_description_msg']['type_description']['fields']

    # Adiciona o index para cada campo
    count = 0
    for field in fields:
        if field['type']['type_id'] in [3,5,7,9, 2,4,6,8, 10,11, 17]:
            field['index'] = count
            count += 1 

    # Renderiza  o template
    template = env.get_template('ufr_enc.jinja.cpp')
    output = template.render({
        'type_name': data_type_name,
        'type_name_cpp': data_type_name.replace('/', '::'),
        'short_name': 'byte',
        'fields': fields
    })
    return output


def render_message_and_write(filename, output_path):
    output = render_message(filename)
    fd = open(output_path, 'w')
    fd.write(output)
    fd.close()


files = list(Path('./sensor_msgs').glob('*.json'))
for file in files:
    output_file = 'out/sensor_msgs/'+file.stem+'.cpp'
    print(file, output_file)
    render_message_and_write(file, output_file)
