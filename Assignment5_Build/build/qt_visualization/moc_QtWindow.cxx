/****************************************************************************
** Meta object code from reading C++ file 'QtWindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../qt_visualization/QtWindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      10,    9,    9,    9, 0x0a,
      33,    9,    9,    9, 0x0a,
      58,    9,    9,    9, 0x0a,
      87,    9,    9,    9, 0x0a,
     111,    9,    9,    9, 0x0a,
     118,    9,    9,    9, 0x0a,
     134,    9,    9,    9, 0x0a,
     142,    9,    9,    9, 0x0a,
     154,    9,    9,    9, 0x0a,
     166,    9,    9,    9, 0x0a,
     189,    9,    9,    9, 0x0a,
     213,    9,    9,    9, 0x0a,
     228,    9,    9,    9, 0x0a,
     255,  250,    9,    9, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_QtWindow[] = {
    "QtWindow\0\0getGoalConfiguration()\0"
    "getRandomConfiguration()\0"
    "getRandomFreeConfiguration()\0"
    "getStartConfiguration()\0open()\0"
    "startPlanning()\0reset()\0saveImage()\0"
    "saveScene()\0setGoalConfiguration()\0"
    "setStartConfiguration()\0toggleCamera()\0"
    "toggleConfiguration()\0doOn\0toggleView(bool)\0"
};

void QtWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QtWindow *_t = static_cast<QtWindow *>(_o);
        switch (_id) {
        case 0: _t->getGoalConfiguration(); break;
        case 1: _t->getRandomConfiguration(); break;
        case 2: _t->getRandomFreeConfiguration(); break;
        case 3: _t->getStartConfiguration(); break;
        case 4: _t->open(); break;
        case 5: _t->startPlanning(); break;
        case 6: _t->reset(); break;
        case 7: _t->saveImage(); break;
        case 8: _t->saveScene(); break;
        case 9: _t->setGoalConfiguration(); break;
        case 10: _t->setStartConfiguration(); break;
        case 11: _t->toggleCamera(); break;
        case 12: _t->toggleConfiguration(); break;
        case 13: _t->toggleView((*reinterpret_cast< const bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QtWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QtWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_QtWindow,
      qt_meta_data_QtWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtWindow))
        return static_cast<void*>(const_cast< QtWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int QtWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
