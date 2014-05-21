/****************************************************************************
** Meta object code from reading C++ file 'object_recorder_panel.h'
**
** Created: Tue May 20 15:04:25 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/object_recorder_panel.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'object_recorder_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_rviz_record_object_pose__ObjectRecorderPanel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      46,   45,   45,   45, 0x08,
      66,   45,   45,   45, 0x08,
      88,   45,   45,   45, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_rviz_record_object_pose__ObjectRecorderPanel[] = {
    "rviz_record_object_pose::ObjectRecorderPanel\0"
    "\0UpdateOutputTopic()\0UpdatePublishButton()\0"
    "Update()\0"
};

void rviz_record_object_pose::ObjectRecorderPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ObjectRecorderPanel *_t = static_cast<ObjectRecorderPanel *>(_o);
        switch (_id) {
        case 0: _t->UpdateOutputTopic(); break;
        case 1: _t->UpdatePublishButton(); break;
        case 2: _t->Update(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData rviz_record_object_pose::ObjectRecorderPanel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject rviz_record_object_pose::ObjectRecorderPanel::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_rviz_record_object_pose__ObjectRecorderPanel,
      qt_meta_data_rviz_record_object_pose__ObjectRecorderPanel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &rviz_record_object_pose::ObjectRecorderPanel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *rviz_record_object_pose::ObjectRecorderPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *rviz_record_object_pose::ObjectRecorderPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_record_object_pose__ObjectRecorderPanel))
        return static_cast<void*>(const_cast< ObjectRecorderPanel*>(this));
    typedef rviz::Panel QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int rviz_record_object_pose::ObjectRecorderPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rviz::Panel QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
