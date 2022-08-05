import importlib

from numpy import typename

""" ros_loader contains methods for dynamically loading ROS message classes at
runtime.  It's achieved by using roslib to load the manifest files for the
package that the respective class is contained in.
Methods typically return the requested class or instance, or None if not found
"""

# Variable containing the loaded classes
_loaded_msgs = {}
_loaded_srvs = {}


class InvalidTypeStringException(Exception):
    def __init__(self, typestring):
        Exception.__init__(self, "%s is not a valid type string" % typestring)


class InvalidModuleException(Exception):
    def __init__(self, modname, subname, original_exception):
        Exception.__init__(
            self,
            "Unable to import %s.%s from package %s. Caused by: %s"
            % (modname, subname, modname, str(original_exception)),
        )


class InvalidClassException(Exception):
    def __init__(self, modname, subname, classname, original_exception):
        Exception.__init__(
            self,
            "Unable to import %s class %s from package %s. Caused by %s"
            % (subname, classname, modname, str(original_exception)),
        )


def get_message_class(typestring):
    """Loads the message type specified.
    Returns the loaded class, or throws exceptions on failure"""
    return _get_msg_class(typestring)


def get_service_class(typestring):
    """Loads the service type specified.
    Returns the loaded class, or None on failure"""
    return _get_srv_class(typestring)


def get_message_instance(typestring):
    """If not loaded, loads the specified type.
    Then returns an instance of it, or None."""
    cls = get_message_class(typestring)
    return cls()


def get_service_request_instance(typestring):
    cls = get_service_class(typestring)
    return cls.Request()


def get_service_response_instance(typestring):
    cls = get_service_class(typestring)
    return cls.Response()


def _get_msg_class(typestring):
    """If not loaded, loads the specified msg class then returns an instance
    of it
    Throws various exceptions if loading the msg class fails"""
    global _loaded_msgs
    try:
        # The type string starts with the package and ends with the
        # class and contains module subnames in between. For
        # compatibility with ROS1 style types, we fall back to use a
        # standard "msg" subname.
        is_action_message = _is_action_msg(typestring)

        splits = [x for x in typestring.split("/") if x]
        if len(splits) > 2:
            subname = ".".join(splits[1:-1])
        else:
            subname = "action" if is_action_message else "msg"

        if is_action_message:
            return _get_action_msg_class(typestring, subname, _loaded_msgs)
        else:
            return _get_class(typestring, subname, _loaded_msgs)

    except (InvalidModuleException, InvalidClassException):
        return _get_class(typestring, "msg", _loaded_msgs)


def _get_srv_class(typestring):
    """If not loaded, loads the specified srv class then returns an instance
    of it
    Throws various exceptions if loading the srv class fails"""
    global _loaded_srvs
    try:
        # The type string starts with the package and ends with the
        # class and contains module subnames in between. For
        # compatibility with ROS1 style types, we fall back to use a
        # standard "srv" subname.
        is_action_service = _is_action_service(typestring)

        splits = [x for x in typestring.split("/") if x]
        if len(splits) > 2:
            subname = ".".join(splits[1:-1])
        else:
            subname = 'action' if is_action_service else "srv"

        if is_action_service:
            return _get_action_service_class(typestring, subname, _loaded_srvs)
        else:
            return _get_class(typestring, subname, _loaded_srvs)

    except (InvalidModuleException, InvalidClassException):
        return _get_class(typestring, "srv", _loaded_srvs)


def _get_class(typestring, subname, cache):
    """If not loaded, loads the specified class then returns an instance
    of it.
    Loaded classes are cached in the provided cache dict
    Throws various exceptions if loading the msg class fails"""

    # First, see if we have this type string cached
    cls = _get_from_cache(cache, typestring)
    if cls is not None:
        return cls

    # Now normalise the typestring
    modname, classname = _splittype(typestring)
    norm_typestring = modname + "/" + classname

    # Check to see if the normalised type string is cached
    cls = _get_from_cache(cache, norm_typestring)
    if cls is not None:
        return cls

    # Load the class
    cls = _load_class(modname, subname, classname)

    # Cache the class for both the regular and normalised typestring
    _add_to_cache(cache, typestring, cls)
    _add_to_cache(cache, norm_typestring, cls)

    return cls


def _load_class(modname, subname, classname):
    """Loads the manifest and imports the module that contains the specified
    type.
    Logic is similar to that of roslib.message.get_message_class, but we want
    more expressive exceptions.
    Returns the loaded module, or None on failure"""

    # This assumes the module is already in the path.
    try:
        pypkg = importlib.import_module(f"{modname}.{subname}")
    except Exception as exc:
        raise InvalidModuleException(modname, subname, exc)

    try:
        return getattr(pypkg, classname)
    except Exception as exc:
        raise InvalidClassException(modname, subname, classname, exc)


def _splittype(typestring):
    """Split the string the / delimiter and strip out empty strings
    Performs similar logic to roslib.names.package_resource_name but is a bit
    more forgiving about excess slashes
    """
    splits = [x for x in typestring.split("/") if x]
    if len(splits) == 3:
        return (splits[0], splits[2])
    if len(splits) == 2:
        return (splits[0], splits[1])
    raise InvalidTypeStringException(typestring)


def _add_to_cache(cache, key, value):
    cache[key] = value


def _get_from_cache(cache, key):
    """Returns the value for the specified key from the cache.
    Locks the lock before doing anything. Returns None if key not in cache"""
    ret = None
    if key in cache:
        ret = cache[key]
    return ret

def _is_action_service(name: str):
    return name.endswith('_SendGoal') or name.endswith('_GetResult') or name.endswith('_CancelGoal')

def _get_action_service_class(typestring: str, subname: str, cache):
    cls = _get_from_cache(cache, typestring)
    if cls is not None:
        return cls

    parts = typestring.split("/")
    action = parts[-1].split('_')
    
    actionName = action[0]
    operationName = action[1]
    modname = parts[0]

    norm_typestring = modname + "/" + actionName + "_" + operationName

    # Check to see if the normalized type string is cached
    cls = _get_from_cache(cache, norm_typestring)
    if cls is not None:
        return cls

    # Load the class
    actionCls = _load_class(modname, subname, actionName)
    cls = getattr(actionCls.Impl, f'{operationName}Service')

    # Cache the class for both the regular and normalized typestring
    _add_to_cache(cache, typestring, cls)
    _add_to_cache(cache, norm_typestring, cls)

    return cls

def _is_action_msg(name: str):
    return name.endswith('_Goal') or name.endswith('_Result') or name.endswith('_FeedbackMessage')

def _get_action_msg_class(typestring: str, subname: str, cache):
    cls = _get_from_cache(cache, typestring)
    if cls is not None:
        return cls

    parts = typestring.split("/")
    action = parts[-1].split('_')
    
    actionName = action[0]
    typeName = action[1]
    modname = parts[0]

    norm_typestring = modname + "/" + actionName + "_" + typeName

    # Check to see if the normalized type string is cached
    cls = _get_from_cache(cache, norm_typestring)
    if cls is not None:
        return cls

    # Load the class
    actionCls = _load_class(modname, subname, actionName)
    cls = getattr(actionCls.Impl, typeName)

    # Cache the class for both the regular and normalized typestring
    _add_to_cache(cache, typestring, cls)
    _add_to_cache(cache, norm_typestring, cls)

    return cls
    