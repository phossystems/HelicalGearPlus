#Author-Ross Korsky 2016-2017
# Released under the MIT licence - see license.txt for full details
#Description-Fission is my effort to making Scripts and Add-ins for Fusion 360 faster and more enjoyable.

import adsk.core, adsk.fusion, adsk.cam, traceback
import re, math, json
from numbers import Number
import time
from collections import OrderedDict


def ObjectCollectionFromList(l):
  oc = adsk.core.ObjectCollection.create()
  for item in l:
    oc.add(item)
  return oc


def ObjectCollectionToList(oc):
  l = []
  for item in oc:
    l.append(item)
  return l

def ConcatenateObjectCollections(l, r):
  for item in r:
    l.add(item)
  return l

class Settings(object):
  def __init__(self, setting_group_name):
    self.__group_name = setting_group_name
    self.__tracked_inputs = dict()

  @property
  def group_name(self):
    assert self.__group_name, 'Settings must have a group name.'
    return self.__group_name

  @property
  def _attributes_collection(self):
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).attributes

  def save(self):
    catch_me = dict()
    for k, v in self.__dict__.items():
      if not re.match( r'^_Settings__.+', k ):
        catch_me[k] = v
    json_value = json.dumps(catch_me, sort_keys=True)
    # adsk.core.Application.get().userInterface.messageBox('Preserving stored values\n' + json_value)
    self._attributes_collection.add(self.group_name, 'settings', json_value)

  def load(self):
    settings_attr = self._attributes_collection.itemByName(self.group_name, 'settings')
    if settings_attr:
      json_value = settings_attr.value
      # adsk.core.Application.get().userInterface.messageBox('Loaded stored values\n' + json_value)
      loaded = json.loads( json_value )
      for k, v in loaded.items():
        self.__dict__[k] = v

  def track(self, input):
    assert input.id not in self.__tracked_inputs, 'Duplicate input ID: ' + input.id
    self.__tracked_inputs[input.id] = input
    if input.id in self.__dict__:
      self._restore(input)
    else:
      self._capture(input)
    return input

  def _restore(self, input):
    class_type = input.classType()
    if hasattr(input, 'expression'):
      input.expression = self.__dict__[input.id]
    elif hasattr(input, 'value'):
      input.value = self.__dict__[input.id]
    elif hasattr(input, 'listItems'):
      selected = self.__dict__[input.id]
      for _item in input.listItems:
        item = adsk.core.ListItem.cast(_item)
        item.isSelected = item.name in selected
    elif hasattr(input, 'formattedText'):
      input.formattedText = self.__dict__[input.id]
    elif class_type == 'adsk::core::GroupCommandInput':
      d = self.__dict__[input.id]
      input.isExpanded = d['is_expanded']
      input.isEnabledCheckBoxDisplayed = d['is_enabled_checkbox_displayed']
      input.isEnabled = d['is_enabled']
    else:
      assert False, 'I dont know how to restore ' + input.id + ' it\'s a ' + input.classType()
    return input

  def _capture(self, input):
    class_type = input.classType()
    if hasattr(input, 'expression'):
      self.__dict__[input.id] = input.expression
    elif hasattr(input, 'value'):
      self.__dict__[input.id] = input.value
    elif hasattr(input, 'listItems'):
      selected = []
      for _item in input.listItems:
        item = adsk.core.ListItem.cast(_item)
        if item.isSelected:
          selected.append(item.name)
      self.__dict__[input.id] = selected
    elif hasattr(input, 'formattedText'):
      self.__dict__[input.id] = input.formattedText
    elif class_type == 'adsk::core::GroupCommandInput':
     self.__dict__[input.id] = {
       'is_expanded': input.isExpanded,
       'is_enabled_checkbox_displayed': input.isEnabledCheckBoxDisplayed,
       'is_enabled': input.isEnabled
     }
    else:
      assert False, 'I dont know how to capture ' + input.id + ' it\'s a ' + input.classType()

    try:
      json.dumps( self.__dict__[input.id] )
    except TypeError:
      err = input.id + ' is not being properly captured'
      if hasattr(input, 'classType'):
        err += '\n  Class Type: ' + input.classType()
      else:
        err += '\n  Type: ' + type(input).__name__

      if hasattr(input, 'expression'):
        err += '\n  expression: ' + input.expression
      if hasattr(input, 'value'):
        err += '\n  value: ' + str(input.value)
      if hasattr(input, 'listItems'):
        err += '\n  listItems: ' + str(input.listItems.count)
      if hasattr(input, 'formattedText'):
        err += '\n  formattedText: ' + input.formattedText

      adsk.core.Application.get().userInterface.messageBox(err)
    return input

  def restore_all(self):
    """Restores tracked input field values from the values held by this settings object."""
    for _, i in self.__tracked_inputs.items():
      self._restore(i)

  def capture_all(self):
    """Captures all tracked input field values into this settings object."""
    for _, i in self.__tracked_inputs.items():
      self._capture(i)


class FusionEventHandler(object):
  """Makes wiering up to Fusion events dead simple and friendly!

  Usage:
    Inherit from this class and call self._auto_wire(command) at some point.
    Annotate the methods you wish to handle events using the return
    annotation to indicate which event to wire to.

  Example Annotation:
    def on_execute(self, args) -> 'execute':
      self.ui.messageBox('EXECUTE')

  NOTICE:
    Only one subscribing method per event source is supported. If the same
    event source is subscribed to multiple times only one method will receive
    events.

  Supported Event Names:
    command_created (may not be autowired)
    destroy
    execute
    activate
    deactivate
    preview
    input_changed
    validate_inputs
    key_down
    key_up
    mouse_click
    mouse_double_click
    mouse_down
    mouse_move
    mouse_up
    mouse_wheel
    mouse_drag_begin
    mouse_drag
    mouse_drag_end
    selecting
  """

  def __init__(self):
    self.__event_handlers = dict()

  def __handler_factory(self, event, callback, handler_cls ):
    """Factory method to create handler classes and bind them to Fusion event sources.

    Args:
      event: object - on which to call .add passing the handler instance
      callback: function - which will be called when the event fires
      handler_cls: type - one of the adsk.core.*EventHandler types

    Returns:
      Instance of a handler if subscription to event was successfull, None otherwise.
    """
    class _Handler(handler_cls):
      def __init__(self):
        super().__init__()

      def notify(self, *args):
        try:
           callback(*args)
        except Exception as ex:
          #adsk.core.Application.get().userInterface.messageBox('Failed:\n{}'.format(traceback.format_exc()))
          adsk.core.Application.get().userInterface.messageBox(
              '{}\n\n--------\n{}'.format(ex, traceback.format_exc()),
              'Error')

    h = _Handler()
    return h if event.add(h) else None

  def _wire_event(self, command, event, callback ):
    """Subscribes a listener to an event trigger.

    See core.py near line 2977

    Args:
      command: adsk.core.Command or adsk.core.CommandDefinitions
      event: string
      callback: Function - which will be called when the event fires
    """
    _wire_handler = self.__handler_factory
    events = {
      'command_created': lambda command, callback: _wire_handler(command.commandCreated, callback, adsk.core.CommandCreatedEventHandler),
      'destroy': lambda command, callback: _wire_handler(command.destroy, callback, adsk.core.CommandEventHandler),
      'execute': lambda command, callback: _wire_handler(command.execute, callback, adsk.core.CommandEventHandler),
      'activate': lambda command, callback: _wire_handler(command.activate, callback, adsk.core.CommandEventHandler),
      'deactivate': lambda command, callback: _wire_handler(command.deactivate, callback, adsk.core.CommandEventHandler),
      'preview': lambda command, callback: _wire_handler(command.executePreview, callback, adsk.core.CommandEventHandler),
      'input_changed': lambda command, callback: _wire_handler(command.inputChanged, callback, adsk.core.InputChangedEventHandler),
      'validate_inputs': lambda command, callback: _wire_handler(command.validateInputs, callback, adsk.core.ValidateInputsEventHandler),
      'key_down': lambda command, callback: _wire_handler(command.keyDown, callback, adsk.core.KeyboardEventHandler),
      'key_up': lambda command, callback: _wire_handler(command.keyUp, callback, adsk.core.KeyboardEventHandler),
      'mouse_click': lambda command, callback: _wire_handler(command.mouseClick, callback, adsk.core.MouseEventHandler),
      'mouse_double_click': lambda command, callback: _wire_handler(command.mouseDoubleClick, callback, adsk.core.MouseEventHandler),
      'mouse_down': lambda command, callback: _wire_handler(command.mouseDown, callback, adsk.core.MouseEventHandler),
      'mouse_move': lambda command, callback: _wire_handler(command.mouseMove, callback, adsk.core.MouseEventHandler),
      'mouse_up': lambda command, callback: _wire_handler(command.mouseUp, callback, adsk.core.MouseEventHandler),
      'mouse_wheel': lambda command, callback: _wire_handler(command.mouseWheel, callback, adsk.core.MouseEventHandler),
      'mouse_drag_begin': lambda command, callback: _wire_handler(command.mouseDragBegin, callback, adsk.core.MouseEventHandler),
      'mouse_drag': lambda command, callback: _wire_handler(command.mouseDrag, callback, adsk.core.MouseEventHandler),
      'mouse_drag_end': lambda command, callback: _wire_handler(command.mouseDragEnd, callback, adsk.core.MouseEventHandler),
      'selecting': lambda command, callback: _wire_handler(command.selectionEvent, callback, adsk.core.SelectionEventHandler)}
    h = events[event](command, callback) if event in events else None

    if h: self.__event_handlers[event] = h

  def _auto_wire(self, command):
    for name in dir(self):
      attr = getattr(self, name)
      if callable(attr) and hasattr(attr,'__annotations__') and 'return' in attr.__annotations__:
        self._wire_event(command, attr.__annotations__['return'], attr)


class CommandBase(FusionEventHandler):
  """Base class for creating Command objects."""
  def __init__(self):
    super().__init__()

  @property
  def app(self):
    return adsk.core.Application.get()

  @property
  def ui(self):
    return self.app.userInterface

  @property
  def command_id(self):
    return type(self).__name__

  @property
  def command_name(self):
    return self.command_id

  @property
  def command_description(self):
    return type(self).__doc__ or ''

  @property
  def is_repeatable(self):
    return True

  def run(self):
    try:
      # resource_dir = os.path.abspath('resources')
      # self.resource_dir = resource_dir if os.path.isdir(resource_dir) else ''
      self._command_definition = self.ui.commandDefinitions.itemById(self.command_id)
      if not self._command_definition:
        self._command_definition = self.add_button()
      assert self._command_definition, 'Script/Addin failed to produce a \'button\''
      self._wire_event(self._command_definition, 'command_created', self.__on_create)

    except:
      self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

  def add_button(self):
    """Called at an appropriate time to add a button to one of Fusions menus.

    Overriders of this method should call super(), then they can add the button to the Fusion menu system if desired.
    Finally the button object must be returned.
    Returns:
      ButtonDefinition
    """
    return self.ui.commandDefinitions.addButtonDefinition(
      self.command_id,
      self.command_name,
      self.command_description,
      self.resource_dir)

  def remove_button(self):
    pass

  def stop(self, context):
    if self._command_definition:
      self._command_definition.deleteMe()
      self.remove_button()

  def __on_create(self, _args):
    args = adsk.core.CommandCreatedEventArgs.cast(_args)
    self._command = args.command
    self._command.isRepeatable = self.is_repeatable
    self._auto_wire(self._command)
    self.settings = Settings(self.command_id)
    self.settings.load()
    self.input_factory = InputFactory(self._command, self.settings)
    self.initialize_inputs(self.input_factory)

  def initialize_inputs(self, factory):
    pass

  def preserve_inputs(self):
    self.settings.capture_all()
    self.settings.save()

class ScriptBase(CommandBase):
  """Base class for scripts, Add-In's should create one or more CommandBase derived classes.
  """
  def __init__(self):
    super().__init__()

  @property
  def is_repeatable(self):
    return False

  def run(self):
    super().run()
    # This is the same is if we had added a button to a menu and the user had clicked it
    self._command_definition.execute()
    # prevent this module from being terminate when the script returns, because we are waiting for event handlers to fire
    adsk.autoTerminate(False)

  def on_destroy(self, args) -> 'destroy':
    adsk.terminate()


class InputFactory(FusionEventHandler):
  """Simplifies the creation and usage of UI input elements (text boxes, buttons, etc).
  """

  def __init__(self, command, settings_manager):
    super().__init__()
    self.__command = adsk.core.Command.cast(command)
    self.__settings_manager = settings_manager
    self.__command_inputs = self.__command.commandInputs
    self.__command_input_stack = []
    self.__change_callbacks = dict()
    self.__validate_callbacks = dict()
    self._design_utils = DesignUtils()
    self._unitsMgr = adsk.core.Application.get().activeProduct.unitsManager
    self._additional_validator = None
    self._auto_wire(self.__command)

  @property
  def additional_validator(self):
    return self._additional_validator

  @additional_validator.setter
  def additional_validator(self, value):
    self._additional_validator = value

  @additional_validator.deleter
  def additional_validator(self):
    del self._additional_validator

  def __on_input_changed(self, _a) -> 'input_changed':
    args = adsk.core.InputChangedEventArgs.cast(_a)
    input = args.input
    if input.id in self.__change_callbacks:
      (cb, i) = self.__change_callbacks[input.id]
      cb(i)

  def __on_validate(self, _a) -> 'validate_inputs':
    args = adsk.core.ValidateInputsEventArgs.cast(_a)
    valid = True
    for input_id, (cb, input) in self.__validate_callbacks.items():
      try:
        valid &= cb(input)
        #if not valid:
        #  adsk.core.Application.get().userInterface.messageBox('{} is not valid'.format(input_id))
      except:  # Exception as ex:
        #adsk.core.Application.get().userInterface.messageBox('Validation of {} raised\n{}'.format(input_id, repr(ex)))
        valid = False
      if not valid: break
    args.areInputsValid &= valid
    if self._additional_validator and callable(self._additional_validator):
      self._additional_validator(args)

  @property
  def settings_manager(self):
    return self.__settings_manager

  @staticmethod
  def _ValueInput(value):
    v = None
    if isinstance(value, Number):
      v = adsk.core.ValueInput.createByReal(value)
    elif isinstance(value, str):
      v = adsk.core.ValueInput.createByString(value)
    else:
      v = adsk.core.ValueInput.createByObject(value)
    return v

  def handle_change(self, input, callback):
    if callback and callable(callback):
      self.__change_callbacks[input.id] = (callback, input)
    elif input.id in self.__change_callbacks:
      del self.__change_callbacks[input.id]

  def handle_validate(self, input, callback):
    """This function will be called OFTEN to validate an inputs value."""
    if callback and callable(callback):
      self.__validate_callbacks[input.id] = (callback, input)
      input.validate = lambda: callback(input)
    elif input.id in self.__validate_callbacks:
      del self.__validate_callbacks[input.id]

  def _set_tips(self, input, name, tooltip, description, help_image):
    if tooltip:
      input.tooltip = tooltip
    elif description:
      input.tooltip = name
    if description: input.tooltipDescription = description
    if help_image: input.toolClipFilename = help_image


  def create_int_spinner(self, id, name, initial_value=0, min=-2147483648, max=2147483647, step=1, tooltip='', description='', help_image=None, on_change=None, on_validate=None, persist=True):
    input = self.__command_inputs.addIntegerSpinnerCommandInput (id, name, min, max, step, initial_value)
    self._set_tips(input, name, tooltip, description, help_image)
    input.eval = lambda: input.value
    self.handle_validate(input, on_validate)
    self.handle_change(input, on_change)
    if persist:
      self.settings_manager.track(input)
    return input

  def addValueInput(self, id, name, initial_value, units, tooltip='', description='', help_image=None, on_change=None, on_validate=None, persist=True):
    iv = InputFactory._ValueInput(initial_value)
    input = self.__command_inputs.addValueInput(id, name, units, iv)
    self._set_tips(input, name, tooltip, description, help_image)
    um = self._unitsMgr
    du = self._design_utils
    input.eval = lambda: um.evaluateExpression(input.expression, units)
    self.handle_change(input, on_change)
    self.handle_validate(input, on_validate)
    if persist:
      self.settings_manager.track(input)
    return input

  def addStringInput(self, id, name, initial_value='', tooltip='', description='', help_image=None, on_change=None, on_validate=None, on_eval=None, persist=True):
    input = self.__command_inputs.addStringValueInput(id, name, str(initial_value))
    self._set_tips(input, name, tooltip, description, help_image)
    self.handle_change(input, on_change)
    self.handle_validate(input, on_validate)
    if on_eval:
      input.eval = lambda: on_eval(input)
    else:
      input.eval = lambda: input.value
    if persist:
      self.settings_manager.track(input)
    return input

  def addSelectionInput(self, id, name, prompt=None, filter=None, tooltip='', description='', help_image=None, on_change=None, on_validate=None, persist=False):
    input = self.__command_inputs.addSelectionInput(id, name, prompt )
    self._set_tips(input, name, tooltip, description, help_image)
    if filter:
      for f in filter:
        input.addSelectionFilter(f)
    self.handle_change(input, on_change)
    self.handle_validate(input, on_validate)
    if persist:
      self.settings_manager.track(input)
    return input

  def create_text_drop_down(self, id, name, items=[], default=None, values=[], tooltip='', description='', help_image=None, on_change=None, on_validate=None, persist=True):
    input = self.__command_inputs.addDropDownCommandInput(id, name, adsk.core.DropDownStyles.TextListDropDownStyle)
    self._set_tips(input, name, tooltip, description, help_image)
    assert len(values) == 0 or len(values) == len(items), 'Number of items and values did not match'
    if len(values) == 0:
      for item in items:
        input.listItems.add(item, item == default)
      input.eval = lambda: input.selectedItem.name
    else:
      value_lookup = dict()
      item_values = zip(items, values)
      for (item, value) in item_values:
        input.listItems.add(item, item == default)
        value_lookup[item] = value
      input.eval = lambda: value_lookup[input.selectedItem.name]
    self.handle_change(input, on_change)
    self.handle_validate(input, on_validate)
    if persist:
      self.settings_manager.track(input)
    return input

  def create_checkbox(self, id, name, initial_value=False, tooltip='', description='', help_image=None, on_change=None, on_validate=None, persist=True):
    input = self.__command_inputs.addBoolValueInput(id, name, True, '', initial_value)
    self._set_tips(input, name, tooltip, description, help_image)
    self.handle_change(input, on_change)
    self.handle_validate(input, on_validate)
    input.eval = lambda: input.value
    if persist:
      self.settings_manager.track(input)
    return input

  def create_textbox(self, id, name='', text='', height=2, read_only=False, tooltip='', description='', help_image=None, on_change=None, on_validate=None, persist=True):
    input = self.__command_inputs.addTextBoxCommandInput(id, name, text, height, read_only)
    self._set_tips(input, name, tooltip, description, help_image)
    self.handle_change(input, on_change)
    self.handle_validate(input, on_validate)
    if persist:
      self.settings_manager.track(input)
    return input

  def begin_group(self, id, name, enabled=True, expanded=False, show_enable_checkbox=False, persist=True):
    input = self.__command_inputs.addGroupCommandInput(id, name)
    input.isEnabled = enabled
    input.isExpanded = expanded
    input.isEnabledCheckBoxDisplayed = show_enable_checkbox
    self.__command_input_stack.append(input.children)
    self.__command_inputs = input.children
    if persist:
      self.settings_manager.track(input)
    return input

  def close_group(self):
    if len(self.__command_input_stack):
      self.__command_inputs = self.__command_input_stack.pop()
    else:
      self.__command_inputs = self.__command.commandInputs

class DesignUtils:
  def __init__(self):
    pass

  @property
  def _app(self):
    return adsk.core.Application.get()

  @property
  def _product(self):
    return self._app.activeProduct

  @property
  def design(self):
    return adsk.fusion.Design.cast(self._product)

  @property
  def root_component(self):
    return self.design.rootComponent

  def CreateNewComponent(self, transform=None):
    if not transform:
      transform = adsk.core.Matrix3D.create()
    occurrence = self.root_component.occurrences.addNewComponent(transform)
    component = occurrence.component
    component.occurrence = occurrence
    return component

  def ActivateRootComponent(self):
    self.design.activateRootComponent()

  def GetUserParameterValue(self, parameter_name):
    """Gets a user parameter value by name.

      Args
        parameter_name (string) parameter name
      Returns
        value (double) or None
    """
    param = self.design.userParameters(parameter_name)
    return param.value if param else None

class SketchHelper(object):
  CLOSE_ENOUGH = 0.00001
  def __init__(self, sketch):
    self.sketch = sketch
    self.points = []
    self.curves = []

  def clear(self):
    self.points.clear()
    self.curves.clear()

  def _find_point(self, point, max_distance=CLOSE_ENOUGH):
    for p in self.points:
      if abs(point.distanceTo(p.geometry)) <= max_distance:
        return p
    return None

  def _find_sketch_point(self, sketch_point, max_distance=CLOSE_ENOUGH):
    return self._find_point(sketch_point.geometry, max_distance)

  def track_point(self, point, reuse=True):
    if reuse:
      p = self._find_point(point)
      if not p:
        p = self.sketch.sketchPoints.add(point)
        self.points.append(p)
    else:
      p = self.sketch.sketchPoints.add(point)
    return p

  def merge_or_track_sketch_point(self, sketch_point):
    tracked_point = self._find_sketch_point(sketch_point)
    if not tracked_point:
      self.points.append(sketch_point)
      tracked_point = sketch_point
    else:
      tracked_point.merge(sketch_point)
    return tracked_point

  def add_line(self, from_point, to_point):
    curve = self.sketch.sketchCurves.sketchLines.addByTwoPoints(
      self.track_point(from_point),
      self.track_point(to_point))
    self.curves.append(curve)
    return curve

  def add_arc_by_center_start_sweep(self, center, start, sweep, expect_end=None):
    curve = self.sketch.sketchCurves.sketchArcs.addByCenterStartSweep(center, start, sweep)
    self.merge_or_track_sketch_point(curve.startSketchPoint)
    self.merge_or_track_sketch_point(curve.endSketchPoint)
    self.curves.append(curve)
    if expect_end:
      assert curve.geometry.endPoint.distanceTo(expect_end) <= SketchHelper.CLOSE_ENOUGH or curve.geometry.startPoint.distanceTo(expect_end) <= SketchHelper.CLOSE_ENOUGH, 'End point of arc was not in the expected location. Expected ({}, {}) was ({}, {})'.format(expect_end.x, expect_end.y, curve.geometry.endPoint.x, curve.geometry.endPoint.y)
    return curve

  def add_arc_by_center_start_sweep_degrees(self, center, start, sweep_degrees, expect_end=None):
    return self.add_arc_by_center_start_sweep(center, start, math.radians(sweep_degrees) , expect_end)

def Point3D(x, y, z=0):
  return adsk.core.Point3D.create(x, y, z)

#
# Profiling tools, etc.
#

class Stopwatch(object):
  def __init__(self):
    self._elapsed = 0
    self._started = -1

  @property
  def elapsed(self):
    return self._elapsed

  def reset(self):
    self.elapsed = 0
    self._started = -1

  def run(self):
    self._started = time.process_time()  # time.perf_counter()


  def pause(self):
    # now = time.perf_counter()
    now = time.process_time()
    if self._started > 0:
      self._elapsed += now - self._started
      self._started = -1


class RelayStopwatch(object):
  def __init__(self):
    self.sections = []
    self.__active_section_start = -1

  def start_section(self, section_name):
    now = time.perf_counter()
    if self.__active_section_start > 0:
      self.sections.append((self.__active_section_name, now - self.__active_section_start))
    self.__active_section_name = section_name
    self.__active_section_start = time.perf_counter()

  def stop(self):
    now = time.perf_counter()
    if self.__active_section_start > 0:
      self.sections.append((self.__active_section_name, now - self.__active_section_start))
      self.__active_section_start = -1

  def __str__(self):
    s = ''
    for name, elapsed in self.sections:
      s += '{}: {}\n'.format(name, elapsed)
    return s


class SegmentedStopwatch(object):
  def __init__(self):
    self._segments = OrderedDict()
    self._active_counts = dict()
    self._active_stopwatch = None

  def switch_segment(self, segment_name):
    if self._active_stopwatch:
      self._active_stopwatch.pause()
    if segment_name not in self._segments:
      self._segments[segment_name] = Stopwatch()
      self._active_counts[segment_name] = 0
    self._active_stopwatch = self._segments[segment_name]
    self._active_counts[segment_name] += 1
    self._active_stopwatch.run()

  def pause(self):
    if self._active_stopwatch:
      self._active_stopwatch.pause()

  def resume(self):
    if self._active_stopwatch:
      self._active_stopwatch.run()

  def stop(self):
    if self._active_stopwatch:
      self._active_stopwatch.pause()
      self._active_stopwatch = None

  def __str__(self):
    s = ''
    total = 0
    for name, stopwatch in self._segments.items():
      s += '{}: {} // {}\n'.format(name, stopwatch.elapsed, self._active_counts[name])
      total += stopwatch.elapsed
    s += 'TOTAL ELAPSED: {}\n'.format(total)
    return s