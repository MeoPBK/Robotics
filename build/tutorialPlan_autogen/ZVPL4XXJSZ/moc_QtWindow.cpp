/****************************************************************************
** Meta object code from reading C++ file 'QtWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "qt_visualization/QtWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QtWindow_t {
    QByteArrayData data[17];
    char stringdata0[240];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QtWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QtWindow_t qt_meta_stringdata_QtWindow = {
    {
QT_MOC_LITERAL(0, 0, 8), // "QtWindow"
QT_MOC_LITERAL(1, 9, 20), // "getGoalConfiguration"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 22), // "getRandomConfiguration"
QT_MOC_LITERAL(4, 54, 26), // "getRandomFreeConfiguration"
QT_MOC_LITERAL(5, 81, 21), // "getStartConfiguration"
QT_MOC_LITERAL(6, 103, 4), // "open"
QT_MOC_LITERAL(7, 108, 13), // "startPlanning"
QT_MOC_LITERAL(8, 122, 5), // "reset"
QT_MOC_LITERAL(9, 128, 9), // "saveImage"
QT_MOC_LITERAL(10, 138, 9), // "saveScene"
QT_MOC_LITERAL(11, 148, 20), // "setGoalConfiguration"
QT_MOC_LITERAL(12, 169, 21), // "setStartConfiguration"
QT_MOC_LITERAL(13, 191, 12), // "toggleCamera"
QT_MOC_LITERAL(14, 204, 19), // "toggleConfiguration"
QT_MOC_LITERAL(15, 224, 10), // "toggleView"
QT_MOC_LITERAL(16, 235, 4) // "doOn"

    },
    "QtWindow\0getGoalConfiguration\0\0"
    "getRandomConfiguration\0"
    "getRandomFreeConfiguration\0"
    "getStartConfiguration\0open\0startPlanning\0"
    "reset\0saveImage\0saveScene\0"
    "setGoalConfiguration\0setStartConfiguration\0"
    "toggleCamera\0toggleConfiguration\0"
    "toggleView\0doOn"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QtWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   84,    2, 0x0a /* Public */,
       3,    0,   85,    2, 0x0a /* Public */,
       4,    0,   86,    2, 0x0a /* Public */,
       5,    0,   87,    2, 0x0a /* Public */,
       6,    0,   88,    2, 0x0a /* Public */,
       7,    0,   89,    2, 0x0a /* Public */,
       8,    0,   90,    2, 0x0a /* Public */,
       9,    0,   91,    2, 0x0a /* Public */,
      10,    0,   92,    2, 0x0a /* Public */,
      11,    0,   93,    2, 0x0a /* Public */,
      12,    0,   94,    2, 0x0a /* Public */,
      13,    0,   95,    2, 0x0a /* Public */,
      14,    0,   96,    2, 0x0a /* Public */,
      15,    1,   97,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   16,

       0        // eod
};

void QtWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QtWindow *>(_o);
        Q_UNUSED(_t)
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

QT_INIT_METAOBJECT const QMetaObject QtWindow::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_QtWindow.data,
    qt_meta_data_QtWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QtWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QtWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QtWindow.stringdata0))
        return static_cast<void*>(this);
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
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 14;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
