import jinja2
import json

# Tipos 
# int   8,16,32,64 :  2,4,6,8
# uint  8,16,32,64 :  3,5,7,9
# float 32,64      :  10,11
# string           :  17


env = jinja2.Environment(loader=jinja2.FileSystemLoader('./templates'))



# fd = open('std_msgs/Byte.json')
fd = open('sensor_msgs/Temperature.json')
raw = fd.read()
fd.close()
msg = json.loads(raw)

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
print(output)