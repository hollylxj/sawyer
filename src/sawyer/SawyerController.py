# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_SawyerController')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_SawyerController')
    _SawyerController = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_SawyerController', [dirname(__file__)])
        except ImportError:
            import _SawyerController
            return _SawyerController
        try:
            _mod = imp.load_module('_SawyerController', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _SawyerController = swig_import_helper()
    del swig_import_helper
else:
    import _SawyerController
del _swig_python_version_info

try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr(self, class_type, name):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    raise AttributeError("'%s' object has no attribute '%s'" % (class_type.__name__, name))


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except __builtin__.Exception:
    class _object:
        pass
    _newclass = 0

class SwigPyIterator(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, SwigPyIterator, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, SwigPyIterator, name)

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _SawyerController.delete_SwigPyIterator
    __del__ = lambda self: None

    def value(self):
        return _SawyerController.SwigPyIterator_value(self)

    def incr(self, n=1):
        return _SawyerController.SwigPyIterator_incr(self, n)

    def decr(self, n=1):
        return _SawyerController.SwigPyIterator_decr(self, n)

    def distance(self, x):
        return _SawyerController.SwigPyIterator_distance(self, x)

    def equal(self, x):
        return _SawyerController.SwigPyIterator_equal(self, x)

    def copy(self):
        return _SawyerController.SwigPyIterator_copy(self)

    def next(self):
        return _SawyerController.SwigPyIterator_next(self)

    def __next__(self):
        return _SawyerController.SwigPyIterator___next__(self)

    def previous(self):
        return _SawyerController.SwigPyIterator_previous(self)

    def advance(self, n):
        return _SawyerController.SwigPyIterator_advance(self, n)

    def __eq__(self, x):
        return _SawyerController.SwigPyIterator___eq__(self, x)

    def __ne__(self, x):
        return _SawyerController.SwigPyIterator___ne__(self, x)

    def __iadd__(self, n):
        return _SawyerController.SwigPyIterator___iadd__(self, n)

    def __isub__(self, n):
        return _SawyerController.SwigPyIterator___isub__(self, n)

    def __add__(self, n):
        return _SawyerController.SwigPyIterator___add__(self, n)

    def __sub__(self, *args):
        return _SawyerController.SwigPyIterator___sub__(self, *args)
    def __iter__(self):
        return self
SwigPyIterator_swigregister = _SawyerController.SwigPyIterator_swigregister
SwigPyIterator_swigregister(SwigPyIterator)

class IntVector(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, IntVector, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, IntVector, name)
    __repr__ = _swig_repr

    def iterator(self):
        return _SawyerController.IntVector_iterator(self)
    def __iter__(self):
        return self.iterator()

    def __nonzero__(self):
        return _SawyerController.IntVector___nonzero__(self)

    def __bool__(self):
        return _SawyerController.IntVector___bool__(self)

    def __len__(self):
        return _SawyerController.IntVector___len__(self)

    def __getslice__(self, i, j):
        return _SawyerController.IntVector___getslice__(self, i, j)

    def __setslice__(self, *args):
        return _SawyerController.IntVector___setslice__(self, *args)

    def __delslice__(self, i, j):
        return _SawyerController.IntVector___delslice__(self, i, j)

    def __delitem__(self, *args):
        return _SawyerController.IntVector___delitem__(self, *args)

    def __getitem__(self, *args):
        return _SawyerController.IntVector___getitem__(self, *args)

    def __setitem__(self, *args):
        return _SawyerController.IntVector___setitem__(self, *args)

    def pop(self):
        return _SawyerController.IntVector_pop(self)

    def append(self, x):
        return _SawyerController.IntVector_append(self, x)

    def empty(self):
        return _SawyerController.IntVector_empty(self)

    def size(self):
        return _SawyerController.IntVector_size(self)

    def swap(self, v):
        return _SawyerController.IntVector_swap(self, v)

    def begin(self):
        return _SawyerController.IntVector_begin(self)

    def end(self):
        return _SawyerController.IntVector_end(self)

    def rbegin(self):
        return _SawyerController.IntVector_rbegin(self)

    def rend(self):
        return _SawyerController.IntVector_rend(self)

    def clear(self):
        return _SawyerController.IntVector_clear(self)

    def get_allocator(self):
        return _SawyerController.IntVector_get_allocator(self)

    def pop_back(self):
        return _SawyerController.IntVector_pop_back(self)

    def erase(self, *args):
        return _SawyerController.IntVector_erase(self, *args)

    def __init__(self, *args):
        this = _SawyerController.new_IntVector(*args)
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this

    def push_back(self, x):
        return _SawyerController.IntVector_push_back(self, x)

    def front(self):
        return _SawyerController.IntVector_front(self)

    def back(self):
        return _SawyerController.IntVector_back(self)

    def assign(self, n, x):
        return _SawyerController.IntVector_assign(self, n, x)

    def resize(self, *args):
        return _SawyerController.IntVector_resize(self, *args)

    def insert(self, *args):
        return _SawyerController.IntVector_insert(self, *args)

    def reserve(self, n):
        return _SawyerController.IntVector_reserve(self, n)

    def capacity(self):
        return _SawyerController.IntVector_capacity(self)
    __swig_destroy__ = _SawyerController.delete_IntVector
    __del__ = lambda self: None
IntVector_swigregister = _SawyerController.IntVector_swigregister
IntVector_swigregister(IntVector)

class DoubleVector(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, DoubleVector, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, DoubleVector, name)
    __repr__ = _swig_repr

    def iterator(self):
        return _SawyerController.DoubleVector_iterator(self)
    def __iter__(self):
        return self.iterator()

    def __nonzero__(self):
        return _SawyerController.DoubleVector___nonzero__(self)

    def __bool__(self):
        return _SawyerController.DoubleVector___bool__(self)

    def __len__(self):
        return _SawyerController.DoubleVector___len__(self)

    def __getslice__(self, i, j):
        return _SawyerController.DoubleVector___getslice__(self, i, j)

    def __setslice__(self, *args):
        return _SawyerController.DoubleVector___setslice__(self, *args)

    def __delslice__(self, i, j):
        return _SawyerController.DoubleVector___delslice__(self, i, j)

    def __delitem__(self, *args):
        return _SawyerController.DoubleVector___delitem__(self, *args)

    def __getitem__(self, *args):
        return _SawyerController.DoubleVector___getitem__(self, *args)

    def __setitem__(self, *args):
        return _SawyerController.DoubleVector___setitem__(self, *args)

    def pop(self):
        return _SawyerController.DoubleVector_pop(self)

    def append(self, x):
        return _SawyerController.DoubleVector_append(self, x)

    def empty(self):
        return _SawyerController.DoubleVector_empty(self)

    def size(self):
        return _SawyerController.DoubleVector_size(self)

    def swap(self, v):
        return _SawyerController.DoubleVector_swap(self, v)

    def begin(self):
        return _SawyerController.DoubleVector_begin(self)

    def end(self):
        return _SawyerController.DoubleVector_end(self)

    def rbegin(self):
        return _SawyerController.DoubleVector_rbegin(self)

    def rend(self):
        return _SawyerController.DoubleVector_rend(self)

    def clear(self):
        return _SawyerController.DoubleVector_clear(self)

    def get_allocator(self):
        return _SawyerController.DoubleVector_get_allocator(self)

    def pop_back(self):
        return _SawyerController.DoubleVector_pop_back(self)

    def erase(self, *args):
        return _SawyerController.DoubleVector_erase(self, *args)

    def __init__(self, *args):
        this = _SawyerController.new_DoubleVector(*args)
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this

    def push_back(self, x):
        return _SawyerController.DoubleVector_push_back(self, x)

    def front(self):
        return _SawyerController.DoubleVector_front(self)

    def back(self):
        return _SawyerController.DoubleVector_back(self)

    def assign(self, n, x):
        return _SawyerController.DoubleVector_assign(self, n, x)

    def resize(self, *args):
        return _SawyerController.DoubleVector_resize(self, *args)

    def insert(self, *args):
        return _SawyerController.DoubleVector_insert(self, *args)

    def reserve(self, n):
        return _SawyerController.DoubleVector_reserve(self, n)

    def capacity(self):
        return _SawyerController.DoubleVector_capacity(self)
    __swig_destroy__ = _SawyerController.delete_DoubleVector
    __del__ = lambda self: None
DoubleVector_swigregister = _SawyerController.DoubleVector_swigregister
DoubleVector_swigregister(DoubleVector)

class SawyerController(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, SawyerController, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, SawyerController, name)
    __repr__ = _swig_repr

    def __init__(self, robot_file):
        this = _SawyerController.new_SawyerController(robot_file)
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this
    __swig_destroy__ = _SawyerController.delete_SawyerController
    __del__ = lambda self: None

    def getXDes(self):
        return _SawyerController.SawyerController_getXDes(self)

    def setXDes(self, x_des):
        return _SawyerController.SawyerController_setXDes(self, x_des)

    def setOriDes(self, ori_des):
        return _SawyerController.SawyerController_setOriDes(self, ori_des)

    def maintainOri(self):
        return _SawyerController.SawyerController_maintainOri(self)

    def setPIDParams(self, kp_pos, kv_pos, kp_ori, kv_ori, kp_joint, kv_joint):
        return _SawyerController.SawyerController_setPIDParams(self, kp_pos, kv_pos, kp_ori, kv_ori, kp_joint, kv_joint)

    def setKMaxVelocity(self, kMaxVelocity):
        return _SawyerController.SawyerController_setKMaxVelocity(self, kMaxVelocity)
    if _newclass:
        getRunloop = staticmethod(_SawyerController.SawyerController_getRunloop)
    else:
        getRunloop = _SawyerController.SawyerController_getRunloop
    if _newclass:
        stop = staticmethod(_SawyerController.SawyerController_stop)
    else:
        stop = _SawyerController.SawyerController_stop

    def calcTorque(self, q, dq, x_des, ori_des):
        return _SawyerController.SawyerController_calcTorque(self, q, dq, x_des, ori_des)

    def testEigen(self, x):
        return _SawyerController.SawyerController_testEigen(self, x)
SawyerController_swigregister = _SawyerController.SawyerController_swigregister
SawyerController_swigregister(SawyerController)
cvar = _SawyerController.cvar
DEBUG = cvar.DEBUG

def SawyerController_getRunloop():
    return _SawyerController.SawyerController_getRunloop()
SawyerController_getRunloop = _SawyerController.SawyerController_getRunloop

def SawyerController_stop(arg2):
    return _SawyerController.SawyerController_stop(arg2)
SawyerController_stop = _SawyerController.SawyerController_stop

# This file is compatible with both classic and new-style classes.


