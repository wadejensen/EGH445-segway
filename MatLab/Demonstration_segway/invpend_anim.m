function [sys,x0,str,ts,simStateCompliance] = invpend_anim(t,x,u,flag,RefBlock,L,r,Ts)
%PENDAN S-function for making pendulum animation.
%
%   See also: PENDDEMO.

%   Copyright 1990-2010 The MathWorks, Inc.
%   $Revision: 1.21.2.4 $

% Plots every major integration step, but has no states of its own
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(RefBlock,L,r,Ts);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u,L,r);

  %%%%%%%%%%%%%%%%
  % Unused flags %
  %%%%%%%%%%%%%%%%
  case { 1, 3, 4, 9 },
    sys = [];
    
  %%%%%%%%%%%%%%%
  % DeleteBlock %
  %%%%%%%%%%%%%%%
  case 'DeleteBlock',
    LocalDeleteBlock
    
  %%%%%%%%%%%%%%%
  % DeleteFigure %
  %%%%%%%%%%%%%%%
  case 'DeleteFigure',
    LocalDeleteFigure
  
  %%%%%%%%%%
  % Slider %
  %%%%%%%%%%
  case 'Slider',
    LocalSlider
  
  %%%%%%%%%
  % Close %
  %%%%%%%%%
  case 'Close',
    LocalClose
  
  %%%%%%%%%%%%
  % Playback %
  %%%%%%%%%%%%
  case 'Playback',
    LocalPlayback
   
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(message('simdemos:general:UnhandledFlag', num2str( flag )));
end

% end pendan

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(RefBlock,L,r,Ts)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 0;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times, for the pendulum demo,
% the animation is updated every 0.1 seconds
%
ts  = [Ts 0];

%
% create the figure, if necessary
%
LocalSegInit(RefBlock,L,r);

% specify that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlUpdate
% Update the pendulum animation.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,L,r) %#ok<INUSL>

fig = get_param(gcbh,'UserData');
if ishghandle(fig, 'figure'),
  if strcmp(get(fig,'Visible'),'on'),
    ud = get(fig,'UserData');
    LocalSegSets(t,ud,u,L,r);
  end
end;
 
sys = [];

% end mdlUpdate

%
%=============================================================================
% LocalDeleteBlock
% The animation block is being deleted, delete the associated figure.
%=============================================================================
%
function LocalDeleteBlock

fig = get_param(gcbh,'UserData');
if ishghandle(fig, 'figure'),
  delete(fig);
  set_param(gcbh,'UserData',-1)
end

% end LocalDeleteBlock

%
%=============================================================================
% LocalDeleteFigure
% The animation figure is being deleted, set the S-function UserData to -1.
%=============================================================================
%
function LocalDeleteFigure

ud = get(gcbf,'UserData');
set_param(ud.Block,'UserData',-1);
  
% end LocalDeleteFigure

%
%=============================================================================
% LocalSlider
% The callback function for the animation window slider uicontrol.  Change
% the reference block's value.
%=============================================================================
%
function LocalSlider

ud = get(gcbf,'UserData');
set_param(ud.RefBlock,'Value',num2str(get(gcbo,'Value')));

% end LocalSlider

%
%=============================================================================
% LocalClose
% The callback function for the animation window close button.  Delete
% the animation figure window.
%=============================================================================
%
function LocalClose

delete(gcbf)

% end LocalClose

%
%=============================================================================
% LocalPlayback
% The callback function for the animation window playback button.  Playback
% the animation.
%=============================================================================
%
function LocalPlayback

%
% first find the animation data in the base workspace, issue an error
% if the information isn't there
%
t = evalin('base','t','[]');
y = evalin('base','y','[]');

if isempty(t) || isempty(y),
  errordlg(...
    ['You must first run the simulation before '...
     'playing back the animation.'],...
    'Animation Playback Error');
end

%
% playback the animation, note that the playback is wrapped in a try-catch
% because is it is possible for the figure and it's children to be deleted
% during the drawnow in LocalPendSets
%
try
  ud = get(gcbf,'UserData');
  for i=1:length(t),
    LocalSegSets(t(i),ud,y(i,:));
  end
catch %#ok<CTCH>
    % do nothing
end

% end LocalPlayback

%
%=============================================================================
% LocalPendSets
% Local function to set the position of the graphics objects in the
% inverted pendulum animation window.
%=============================================================================
%
function LocalSegSets(time,ud,u,L,r)

XDelta   = 0.02;
PDelta   = 0.01;
TDelta   = 0.1;
XCart = u(1);
angle = pi/2-u(2);
WheelAngle = -u(1)/r;
XPendTop = XCart + 2*L*cos(angle);
YPendTop = r+2*L*sin(angle);
PDcosT   = PDelta*cos(angle);
PDsinT   = -PDelta*sin(angle);

[WheelX, WheelY] = LocalCircle(XCart, r, r, 100);

set(ud.Cart,...
  'XData', WheelX,...
  'YData', WheelY);
set(ud.Pend,...
  'XData',[XPendTop-PDsinT XPendTop+PDsinT; XCart-PDsinT XCart+PDsinT], ...
  'YData',[YPendTop-PDcosT YPendTop+PDcosT; r-PDcosT r+PDcosT]);
set(ud.TimeField,...
  'String',num2str(time));
set(ud.RefMark,...
  'XData',u(3)+[-XDelta 0 XDelta]);
set(ud.AngleMark,...
  'XData', [XCart XCart+r*cos(WheelAngle+TDelta) XCart+r*cos(WheelAngle-TDelta)],...
  'YData', [r r+r*sin(WheelAngle+TDelta) r+r*sin(WheelAngle-TDelta)])

% Force plot to be drawn
pause(0)
drawnow

% end LocalPendSets

%
%=============================================================================
% LocalPendInit
% Local function to initialize the pendulum animation.  If the animation
% window already exists, it is brought to the front.  Otherwise, a new
% figure window is created.
%=============================================================================
%
function LocalSegInit(RefBlock,L,r)

%
% The name of the reference is derived from the name of the
% subsystem block that owns the pendulum animation S-function block.
% This subsystem is the current system and is assumed to be the same
% layer at which the reference block resides.
%
sys = get_param(gcs,'Parent');

TimeClock = 0;
RefSignal = str2double(get_param([sys '/' RefBlock],'Value'));
XCart     = 0;
Theta     = 0;

XDelta    = 0.02;
PDelta    = 0.01;
TDelta    = 0.1;
XPendTop  = XCart + 2*L*sin(Theta); % Will be zero
YPendTop  = r+2*L*cos(Theta);         % Will be 10
PDcosT    = PDelta*cos(Theta);     % Will be 0.2
PDsinT    = -PDelta*sin(Theta);    % Will be zero

[WheelX, WheelY] = LocalCircle(XCart, r, r, 100);

%
% The animation figure handle is stored in the pendulum block's UserData.
% If it exists, initialize the reference mark, time, cart, and pendulum
% positions/strings/etc.
%
Fig = get_param(gcbh,'UserData');
if ishghandle(Fig ,'figure'),
  FigUD = get(Fig,'UserData');
  set(FigUD.RefMark,...
      'XData',RefSignal+[-XDelta 0 XDelta]);
  set(FigUD.AngleMark,...
      'XData', [XCart XCart+r*cos(Theta+TDelta) XCart+r*cos(Theta-TDelta)],...
      'YData', [r r+r*sin(Theta+TDelta) r+r*sin(Theta-TDelta)]);
  set(FigUD.TimeField,...
      'String',num2str(TimeClock));
  set(FigUD.Pend,...
      'XData',[XPendTop-PDsinT XPendTop+PDsinT; XCart-PDsinT XCart+PDsinT], ...
      'YData',[YPendTop-PDcosT YPendTop+PDcosT; r-PDcosT r+PDcosT]);
  set(FigUD.Cart,...
      'XData', WheelX, ...
      'YData', WheelY);
      
  %
  % bring it to the front
  %
  figure(Fig);
  return
end

%
% the animation figure doesn't exist, create a new one and store its
% handle in the animation block's UserData
%
FigureName = 'Pendulum Visualization';
Fig = figure(...
  'Units',           'pixel',...
  'Position',        [100 100 900 500],...
  'Name',            FigureName,...
  'NumberTitle',     'off',...
  'IntegerHandle',   'off',...
  'HandleVisibility','callback',...
  'Resize',          'off',...
  'DeleteFcn',       'invpend_anim([],[],[],''DeleteFigure'')',...
  'CloseRequestFcn', 'invpend_anim([],[],[],''Close'');');
AxesH = axes(...
  'Parent',  Fig,...
  'Units',   'pixel',...
  'Position',[00 50 900 338],...
  'CLim',    [1 64], ...
  'Xlim',    [-0.8 0.8],...
  'Ylim',    [-0.05 0.55],...
  'Visible', 'off');
Pend = surface(...
  'Parent',   AxesH,...
  'XData',    [XPendTop-PDcosT XPendTop+PDcosT; XCart-PDcosT XCart+PDcosT], ...
  'YData',    [YPendTop-PDsinT YPendTop+PDsinT; r-PDsinT r+PDsinT], ...
  'ZData',    zeros(2),...
  'CData',    11*ones(2));
Cart = patch(...
  'Parent',   AxesH,...
  'XData',    WheelX,...
  'YData',    WheelY,...
  'CData',    11*ones(size(WheelX)),...
  'FaceColor','flat');
RefMark = patch(...
  'Parent',   AxesH,...
  'XData',    RefSignal+[-XDelta 0 XDelta],...
  'YData',    [-0.01 0 -0.01],...
  'CData',    22,...
  'FaceColor','flat');
AngleMark = patch(...
  'Parent', AxesH,...
  'XData', [XCart XCart+r*cos(Theta+TDelta) XCart+r*cos(Theta-TDelta)],...
  'YData', [r r+r*sin(Theta+TDelta) r+r*sin(Theta-TDelta)],...
  'CData', 22, ...
  'FaceColor','flat');
uicontrol(...
  'Parent',  Fig,...
  'Style',   'text',...
  'Units',   'pixel',...
  'Position',[0 0 900 50]);
uicontrol(...
  'Parent',             Fig,...
  'Style',              'text',...
  'Units',              'pixel',...
  'Position',           [350 0 100 25], ...
  'HorizontalAlignment','right',...
  'String',             'Time: ');
TimeField = uicontrol(...
  'Parent',             Fig,...
  'Style',              'text',...
  'Units',              'pixel', ...
  'Position',           [450 0 100 25],...
  'HorizontalAlignment','left',...
  'String',             num2str(TimeClock));
SlideControl = uicontrol(...
  'Parent',   Fig,...
  'Style',    'slider',...
  'Units',    'pixel', ...
  'Position', [100 25 700 22],...
  'Min',      -0.5,...
  'Max',      0.5,...
  'Value',    RefSignal,...
  'Callback', 'invpend_anim([],[],[],''Slider'');');
uicontrol(...
  'Parent',  Fig,...
  'Style',   'pushbutton',...
  'Position',[815 15 70 20],...
  'String',  'Close', ...
  'Callback','invpend_anim([],[],[],''Close'');');
uicontrol(...
  'Parent',  Fig,...
  'Style',   'pushbutton',...
  'Position',[15 15 70 20],...
  'String',  'Playback', ...
  'Callback','invpend_anim([],[],[],''Playback'');',...
  'Interruptible','off',...
  'BusyAction','cancel');

%
% all the HG objects are created, store them into the Figure's UserData
%
FigUD.Pend         = Pend;
FigUD.Cart         = Cart;
FigUD.TimeField    = TimeField;
FigUD.SlideControl = SlideControl;
FigUD.RefMark      = RefMark;
FigUD.AngleMark    = AngleMark;
FigUD.Block        = get_param(gcbh,'Handle');
FigUD.RefBlock     = get_param([sys '/' RefBlock],'Handle');
set(Fig,'UserData',FigUD);

drawnow

%
% store the figure handle in the animation block's UserData
%
set_param(gcbh,'UserData',Fig);

% end LocalPendInit

function [X,Y]=LocalCircle(x,y,r,pnts)
X = zeros(pnts,1);
Y = zeros(pnts,1);
for i = 1:pnts
    X(i) = x + r*cos(i/pnts*2*pi);
    Y(i) = y + r*sin(i/pnts*2*pi);
end