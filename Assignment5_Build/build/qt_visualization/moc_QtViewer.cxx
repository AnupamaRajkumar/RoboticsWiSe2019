/****************************************************************************
** Meta object code from reading C++ file 'QtViewer.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../qt_visualization/QtViewer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtViewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtViewer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   10,    9,    9, 0x0a,
      57,   48,    9,    9, 0x0a,
     123,  119,    9,    9, 0x2a,
     185,  180,    9,    9, 0x0a,
     236,  229,    9,    9, 0x0a,
     283,   10,    9,    9, 0x2a,
     335,  325,    9,    9, 0x0a,
     383,  379,    9,    9, 0x0a,
     425,  411,    9,    9, 0x0a,
     469,  180,    9,    9, 0x0a,
     509,  507,    9,    9, 0x0a,
     539,  119,    9,    9, 0x0a,
     587,  180,    9,    9, 0x0a,
     622,   10,    9,    9, 0x0a,
     655,    9,    9,    9, 0x0a,
     663,    9,    9,    9, 0x0a,
     676,    9,    9,    9, 0x0a,
     689,    9,    9,    9, 0x0a,
     703,    9,    9,    9, 0x0a,
     718,    9,    9,    9, 0x0a,
     742,  734,    9,    9, 0x0a,
     776,  767,    9,    9, 0x0a,
     795,  767,    9,    9, 0x0a,
     819,  814,    9,    9, 0x0a,
     850,  814,    9,    9, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_QtViewer[] = {
    "QtViewer\0\0q\0drawConfiguration(rl::math::Vector)\0"
    "u,v,free\0"
    "drawConfigurationEdge(rl::math::Vector,rl::math::Vector,bool)\0"
    "u,v\0drawConfigurationEdge(rl::math::Vector,rl::math::Vector)\0"
    "path\0drawConfigurationPath(rl::plan::VectorList)\0"
    "q,free\0drawConfigurationVertex(rl::math::Vector,bool)\0"
    "drawConfigurationVertex(rl::math::Vector)\0"
    "xyz0,xyz1\0drawLine(rl::math::Vector,rl::math::Vector)\0"
    "xyz\0drawPoint(rl::math::Vector)\0"
    "center,radius\0drawSphere(rl::math::Vector,rl::math::Real)\0"
    "drawSweptVolume(rl::plan::VectorList)\0"
    "t\0drawWork(rl::math::Transform)\0"
    "drawWorkEdge(rl::math::Vector,rl::math::Vector)\0"
    "drawWorkPath(rl::plan::VectorList)\0"
    "drawWorkVertex(rl::math::Vector)\0"
    "reset()\0resetEdges()\0resetLines()\0"
    "resetPoints()\0resetSpheres()\0"
    "resetVertices()\0message\0"
    "showMessage(std::string)\0filename\0"
    "saveImage(QString)\0saveScene(QString)\0"
    "doOn\0toggleConfigurationEdges(bool)\0"
    "toggleConfigurationVertices(bool)\0"
};

void QtViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QtViewer *_t = static_cast<QtViewer *>(_o);
        switch (_id) {
        case 0: _t->drawConfiguration((*reinterpret_cast< const rl::math::Vector(*)>(_a[1]))); break;
        case 1: _t->drawConfigurationEdge((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Vector(*)>(_a[2])),(*reinterpret_cast< const bool(*)>(_a[3]))); break;
        case 2: _t->drawConfigurationEdge((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Vector(*)>(_a[2]))); break;
        case 3: _t->drawConfigurationPath((*reinterpret_cast< const rl::plan::VectorList(*)>(_a[1]))); break;
        case 4: _t->drawConfigurationVertex((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const bool(*)>(_a[2]))); break;
        case 5: _t->drawConfigurationVertex((*reinterpret_cast< const rl::math::Vector(*)>(_a[1]))); break;
        case 6: _t->drawLine((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Vector(*)>(_a[2]))); break;
        case 7: _t->drawPoint((*reinterpret_cast< const rl::math::Vector(*)>(_a[1]))); break;
        case 8: _t->drawSphere((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Real(*)>(_a[2]))); break;
        case 9: _t->drawSweptVolume((*reinterpret_cast< const rl::plan::VectorList(*)>(_a[1]))); break;
        case 10: _t->drawWork((*reinterpret_cast< const rl::math::Transform(*)>(_a[1]))); break;
        case 11: _t->drawWorkEdge((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Vector(*)>(_a[2]))); break;
        case 12: _t->drawWorkPath((*reinterpret_cast< const rl::plan::VectorList(*)>(_a[1]))); break;
        case 13: _t->drawWorkVertex((*reinterpret_cast< const rl::math::Vector(*)>(_a[1]))); break;
        case 14: _t->reset(); break;
        case 15: _t->resetEdges(); break;
        case 16: _t->resetLines(); break;
        case 17: _t->resetPoints(); break;
        case 18: _t->resetSpheres(); break;
        case 19: _t->resetVertices(); break;
        case 20: _t->showMessage((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 21: _t->saveImage((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 22: _t->saveScene((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 23: _t->toggleConfigurationEdges((*reinterpret_cast< const bool(*)>(_a[1]))); break;
        case 24: _t->toggleConfigurationVertices((*reinterpret_cast< const bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QtViewer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QtViewer::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_QtViewer,
      qt_meta_data_QtViewer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtViewer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtViewer))
        return static_cast<void*>(const_cast< QtViewer*>(this));
    if (!strcmp(_clname, "rl::plan::Viewer"))
        return static_cast< rl::plan::Viewer*>(const_cast< QtViewer*>(this));
    return QWidget::qt_metacast(_clname);
}

int QtViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
