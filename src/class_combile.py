cl1 = ['casa', 'carro']
cl2 = ['casa', 'bicicleta']

def create_ob(cl):
    ob = {}
    for i in range(len(cl)):
        ob.update({i: {
            'name':cl[i],
            'server_id':-1
        }})
    return ob

dt1={
    'name': 'd1',
    'id': 1,
    'type': 'object',
    'architecture': 'yolov5',
    'n_classes': 2,
    'classes': create_ob(cl1)
}

dt2={
    'name': 'd2',
    'id': 2,
    'type': 'object',
    'architecture': 'yolov5',
    'n_classes': 2,
    'classes': create_ob(cl2)
}


detectors={}
detectors.update({dt1['id']:dt1})
detectors.update({dt2['id']:dt2})

print(detectors[1])

print(detectors[len(detectors)])

classes={}

n_classes = 0

def add_dt(dt):
    n_classes = len(classes)

    if not classes:
        for i in range(len(dt['classes'])):
            classes.update({i:dt['classes'][i]['name']})
            n_classes += 1
    else:
        for i in range(len(dt['classes'])):
            # check if exist
            rep = False
            for j in range(n_classes):
                if classes[j] == dt['classes'][i]['name']:
                    rep = True     

            # append
            if not rep:
                classes.update({len(classes):dt['classes'][i]['name']})
                n_classes += 1

add_dt(detectors[1])
add_dt(detectors[2])
print(classes)

for i in detectors:
    print(detectors[i])


#print(dt1['classes']['class_id_detector'])

#classes = {}
#
#print(len(classes))
#
#if classes:
#    print('a')
#    print(classes)
#else:
#    print(classes)






#fstab
#LABEL=chopin_02 /mnt/chopin_02 auto nosuid,nodev,nofail,x-gvfs-show 0 0