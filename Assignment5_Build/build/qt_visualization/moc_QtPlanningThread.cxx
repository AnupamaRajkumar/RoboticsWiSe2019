/****************************************************************************
** Meta object code from reading C++ file 'QtPlanningThread.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../qt_visualization/QtPlanningThread.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtPlanningThread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtPlanningThread[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      19,       // signalCount

 // signals: signature, parameters, type, tag, flags
      20,   18,   17,   17, 0x05,
      72,   61,   17,   17, 0x05,
     146,  139,   17,   17, 0x05,
     203,  198,   17,   17, 0x05,
     252,   17,   17,   17, 0x05,
     283,  273,   17,   17, 0x05,
     332,   17,   17,   17, 0x05,
     353,   17,   17,   17, 0x05,
     384,  370,   17,   17, 0x05,
     433,   17,   17,   17, 0x05,
     460,  456,   17,   17, 0x05,
     493,   17,   17,   17, 0x05,
     515,  198,   17,   17, 0x05,
     558,   17,   17,   17, 0x05,
     589,  581,   17,   17, 0x05,
     633,  631,   17,   17, 0x05,
     674,  668,   17,   17, 0x05,
     727,  198,   17,   17, 0x05,
     767,   18,   17,   17, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_QtPlanningThread[] = {
    "QtPlanningThread\0\0q\0"
    "configurationRequested(rl::math::Vector)\0"
    "q0,q1,free\0"
    "configurationEdgeRequested(rl::math::Vector,rl::math::Vector,bool)\0"
    "q,free\0configurationVertexRequested(rl::math::Vector,bool)\0"
    "path\0configurationPathRequested(rl::plan::VectorList)\0"
    "edgeResetRequested()\0xyz0,xyz1\0"
    "lineRequested(rl::math::Vector,rl::math::Vector)\0"
    "lineResetRequested()\0resetRequested()\0"
    "center,radius\0"
    "sphereRequested(rl::math::Vector,rl::math::Real)\0"
    "sphereResetRequested()\0xyz\0"
    "pointRequested(rl::math::Vector)\0"
    "pointResetRequested()\0"
    "sweptVolumeRequested(rl::plan::VectorList)\0"
    "vertexResetRequested()\0message\0"
    "showMessageRequested(const::std::string&)\0"
    "t\0workRequested(rl::math::Transform)\0"
    "q0,q1\0workEdgeRequested(rl::math::Vector,rl::math::Vector)\0"
    "workPathRequested(rl::plan::VectorList)\0"
    "workVertexRequested(rl::math::Vector)\0"
};

void QtPlanningThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QtPlanningThread *_t = static_cast<QtPlanningThread *>(_o);
        switch (_id) {
        case 0: _t->configurationRequested((*reinterpret_cast< const rl::math::Vector(*)>(_a[1]))); break;
        case 1: _t->configurationEdgeRequested((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Vector(*)>(_a[2])),(*reinterpret_cast< const bool(*)>(_a[3]))); break;
        case 2: _t->configurationVertexRequested((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const bool(*)>(_a[2]))); break;
        case 3: _t->configurationPathRequested((*reinterpret_cast< const rl::plan::VectorList(*)>(_a[1]))); break;
        case 4: _t->edgeResetRequested(); break;
        case 5: _t->lineRequested((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Vector(*)>(_a[2]))); break;
        case 6: _t->lineResetRequested(); break;
        case 7: _t->resetRequested(); break;
        case 8: _t->sphereRequested((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Real(*)>(_a[2]))); break;
        case 9: _t->sphereResetRequested(); break;
        case 10: _t->pointRequested((*reinterpret_cast< const rl::math::Vector(*)>(_a[1]))); break;
        case 11: _t->pointResetRequested(); break;
        case 12: _t->sweptVolumeRequested((*reinterpret_cast< const rl::plan::VectorList(*)>(_a[1]))); break;
        case 13: _t->vertexResetRequested(); break;
        case 14: _t->showMessageRequested((*reinterpret_cast< const::std::string(*)>(_a[1]))); break;
        case 15: _t->workRequested((*reinterpret_cast< const rl::math::Transform(*)>(_a[1]))); break;
        case 16: _t->workEdgeRequested((*reinterpret_cast< const rl::math::Vector(*)>(_a[1])),(*reinterpret_cast< const rl::math::Vector(*)>(_a[2]))); break;
        case 17: _t->workPathRequested((*reinterpret_cast< const rl::plan::VectorList(*)>(_a[1]))); break;
        case 18: _t->workVertexRequested((*reinterpret_cast< const rl::math::Vector(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QtPlanningThread::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QtPlanningThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_QtPlanningThread,
      qt_meta_data_QtPlanningThread, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtPlanningThread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtPlanningThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtPlanningThread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtPlanningThread))
        return static_cast<void*>(const_cast< QtPlanningThread*>(this));
    if (!strcmp(_clname, "rl::plan::Viewer"))
        return static_cast< rl::plan::Viewer*>(const_cast< QtPlanningThread*>(this));
    return QThread::qt_metacast(_clname);
}

int QtPlanningThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    }
    return _id;
}

// SIGNAL 0
void QtPlanningThread::configurationRequested(const rl::math::Vector & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QtPlanningThread::configurationEdgeRequested(const rl::math::Vector & _t1, const rl::math::Vector & _t2, const bool & _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QtPlanningThread::configurationVertexRequested(const rl::math::Vector & _t1, const bool & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void QtPlanningThread::configurationPathRequested(const rl::plan::VectorList & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void QtPlanningThread::edgeResetRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void QtPlanningThread::lineRequested(const rl::math::Vector & _t1, const rl::math::Vector & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void QtPlanningThread::lineResetRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 6, 0);
}

// SIGNAL 7
void QtPlanningThread::resetRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 7, 0);
}

// SIGNAL 8
void QtPlanningThread::sphereRequested(const rl::math::Vector & _t1, const rl::math::Real & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void QtPlanningThread::sphereResetRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 9, 0);
}

// SIGNAL 10
void QtPlanningThread::pointRequested(const rl::math::Vector & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void QtPlanningThread::pointResetRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 11, 0);
}

// SIGNAL 12
void QtPlanningThread::sweptVolumeRequested(const rl::plan::VectorList & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void QtPlanningThread::vertexResetRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 13, 0);
}

// SIGNAL 14
void QtPlanningThread::showMessageRequested(const ::std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 14, _a);
}

// SIGNAL 15
void QtPlanningThread::workRequested(const rl::math::Transform & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 15, _a);
}

// SIGNAL 16
void QtPlanningThread::workEdgeRequested(const rl::math::Vector & _t1, const rl::math::Vector & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 16, _a);
}

// SIGNAL 17
void QtPlanningThread::workPathRequested(const rl::plan::VectorList & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 17, _a);
}

// SIGNAL 18
void QtPlanningThread::workVertexRequested(const rl::math::Vector & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 18, _a);
}
QT_END_MOC_NAMESPACE
