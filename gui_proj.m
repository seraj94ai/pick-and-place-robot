function varargout = gui_proj(varargin)
% GUI_PROJ MATLAB code for gui_proj.fig
%      GUI_PROJ, by itself, creates a new GUI_PROJ or raises the existing
%      singleton*.
%
%      H = GUI_PROJ returns the handle to a new GUI_PROJ or the handle to
%      the existing singleton*.
%
%      GUI_PROJ('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_PROJ.M with the given input arguments.
%
%      GUI_PROJ('Property','Value',...) creates a new GUI_PROJ or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_proj_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_proj_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_proj

% Last Modified by GUIDE v2.5 21-May-2022 07:28:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_proj_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_proj_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before gui_proj is made visible.
function gui_proj_OpeningFcn(hObject, eventdata, handles, varargin)
global sliderValue1 sliderValue2 sliderValue3 sliderValue4

% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_proj (see VARARGIN)

% Choose default command line output for gui_proj
handles.output = hObject;

% clc; 
% clear; 
% L(1) = Revolute('d', 0,'a', 0,'alpha', pi/2,'qlim',[0 2*pi])
% L(2) = Revolute('d', 0,'a', 10,'alpha', 0,'qlim',[0 pi])
% L(3) = Revolute('d', 0,'a', 10,'alpha', 0,'qlim',[0 pi])
% L(4) = Revolute('d', 0,'a', 4,'alpha', pi/2,'qlim',[0 pi])

%%% Now, we define the five links 
L(1) = Link([0, 1.5, 0, pi/2]);%,'qlim',[-pi pi]);
L(2) = Link([0, 3.4, 12.5, 0]);%,'qlim',[0 pi]);
L(3) = Link([0, -2.6, 10, 0]);%,'qlim',[0 pi]);
L(4) = Link([0, 0.6, 0, -pi/2]);%,'qlim',[0 pi]);

L(5) = Link([0, 0, 0, pi/2]);%,'qlim',[0 pi]);
L(6) = Link([0, -0.6, 0, 0]);%,'qlim',[0 pi]);
% 
%  L(1).qlim=[deg2rad(0) deg2rad(360)];
%  L(2).qlim=[deg2rad(90) deg2rad(180)];
%  L(3).qlim=[deg2rad(0) deg2rad(170)];
%  L(4).qlim=[deg2rad(20) deg2rad(90)];
%  
Robot = SerialLink(L); 
Robot.name = 'ABX'; 
%Robot.base = [1 0 0 0;0 1 0 0;0 0 1 0; 0 0 0 1];
%%% We represent the robot to check if it has correctly been defined 
Robot.ikineType = 'nooffset';
sliderValue1 = 0;
sliderValue2 = 0;
sliderValue3 = -90;
% sliderValue4 = 0;
% sliderValue1 = 50;
% sliderValue2 = 110;
% sliderValue3 = 30;
% sliderValue4 = 30;
qq = [sliderValue1*pi/180 sliderValue2*pi/180 sliderValue3*pi/180 0 0 0]
Robot.plot(qq); 
global first_place
first_place = Robot.fkine(qq)
first_place = transl(first_place)
set(handles.slider1,'Value', sliderValue1); 
set(handles.slider2,'Value', sliderValue2); 
set(handles.slider3,'Value', sliderValue3); 
% set(handles.slider4,'Value', sliderValue4); 

set(handles.edit1,'String', num2str(sliderValue1)); 
set(handles.edit2,'String', num2str(sliderValue2)); 
set(handles.edit3,'String', num2str(sliderValue3)); 
% set(handles.edit4,'String', num2str(sliderValue4)); 
global plott
plott = 1

x = 25;
y = 25;
z = -10;
x_ = [-x -x x x];
y_ = [y -y -y y];
z_ = [z z z z];
patch(x_,y_,z_,'green');
global target 
global arr_sph arr_sph_c
arr_sph_c = {}
arr_sph = []

%------------------------------------
hold on
%------------------------------------
xc = -12;
yc = -12;
zc = -5;
L = 7;
target = [xc,yc,zc];
alpha = 0.8
Xc_ = [0 0 0 0 0 1;1 0 1 1 1 1 ;1 0 1 1 1 1;0 0 0 0 0 1];
Yc_ = [0 0 0 0 1 0;0 1 0 0 1 1 ;0 1 1 1 1 1;0 0 1 1 1 0];
Zc_ = [0 0 1 0 0 1;0 0 1 0 0 0 ;1 1 1 0 1 1;1 1 1 0 1 1];
c = 'blue'
Xc_ = L*(Xc_ - 0.5) + xc;
Yc_ = L*(Yc_ - 0.5) + yc;
Zc_ = L*(Zc_ - 0.5) + zc;
fill3(Xc_,Yc_,Zc_,c,'FaceAlpha',alpha);
hold off
%------------------------------------
%plot3d([],[],[z z z z])
% L_1 = 20;
% L_2 = 50;
% L_3 = 40;
% 
% L (1) = Link([0 L_1 0 pi/2]);
% L (2) = Link([0 0 L_2 0]);
% L (3) = Link([0 0 L_3 0]);
% L(3).offset = -pi/2;
% 
% Robot = SerialLink(L);
% Robot.name = 'RRR_Robot';
% 
% handles.Theta_1.String = '90';
% handles.Theta_2.String = '90';
% handles.Theta_3.String = '90';
% 
% J = Robot.fkine([ pi/2 pi/2 pi/2]);
% 
% handles.CurrentT1 = pi/2;
% handles.CurrentT2 = pi/2;
% handles.CurrentT3 = pi/2;
% 
% handles.Pos_X.String = num2str(floor(J(1,4)));
% handles.Pos_Y.String = num2str(floor(J(2,4)));
% handles.Pos_Z.String = num2str(floor(J(3,4)));
% 
% Robot.plot([pi/2 pi/2 pi/2]);
% view(45,30);


handles.Robot = Robot;
% handles.sliderValue1 = sliderValue1;
% handles.sliderValue2 = sliderValue2;
% handles.sliderValue3 = sliderValue3;
% handles.sliderValue4 = sliderValue4;
% Update handles structure
% CallBack = @(~,b) disp(b.AffectedObject.Value);
% CallBack1 = @(~,b) slider1_Callback(hObject, b, handles)
% CallBack2 = @(~,b) slider2_Callback(hObject, b, handles)
% CallBack3 = @(~,b) slider3_Callback(hObject, b, handles)
% CallBack4 = @(~,b) slider4_Callback(hObject, b, handles)
% % F = figure();
% % H = uicontrol(F,'Style','slider');
% addlistener(handles.slider1, 'Value', 'PostSet',CallBack1);
% addlistener(handles.slider2, 'Value', 'PostSet',CallBack2);
% addlistener(handles.slider3, 'Value', 'PostSet',CallBack3);
% addlistener(handles.slider4, 'Value', 'PostSet',CallBack4);

guidata(hObject, handles);

% Update handles structure

% UIWAIT makes gui_proj wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% $$$$$$$$$$$$
% waypoints = [1 -1 -1 ; 1 2 1;3 -2 1;3 2 -1];
% trajectory = cscvn(waypoints');
% fnplt(trajectory,'r',2)
% 
% $$$$$$$$$$$$
% --- Outputs from this function are returned to the command line.
function varargout = gui_proj_OutputFcn(~, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, ~, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sliderValue1 sliderValue2 sliderValue3 sliderValue4

Robot = handles.Robot
% sliderValue1 = handles.sliderValue1 ;
% sliderValue2 = handles.sliderValue2 ;
% sliderValue3 = handles.sliderValue3 ;
% sliderValue4 = handles.sliderValue4 ;
sliderValue1 = get(handles.slider1,'Value');
sliderValue2 = get(handles.slider2,'Value');
sliderValue3 = get(handles.slider3,'Value');
% sliderValue4 = get(handles.slider4,'Value');
set(handles.edit1,'String', num2str(sliderValue1)); 
Robot.plot([sliderValue1*pi/180  sliderValue2*pi/180 sliderValue3*pi/180 0  0 0]); 
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% sliderValue1 = handles.sliderValue1 ;
% sliderValue2 = handles.sliderValue2 ;
% sliderValue3 = handles.sliderValue3 ;
% sliderValue4 = handles.sliderValue4 ;

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Robot = handles.Robot
% handles.sliderValue1 = sliderValue1;
% handles.sliderValue2 = sliderValue2;
% handles.sliderValue3 = sliderValue3;
% handles.sliderValue4 = sliderValue4;
global sliderValue1 sliderValue2 sliderValue3 sliderValue4

sliderValue1 = get(handles.slider1,'Value');
sliderValue2 = get(handles.slider2,'Value');
sliderValue3 = get(handles.slider3,'Value');
% sliderValue4 = get(handles.slider4,'Value');set(handles.edit2,'String', num2str(sliderValue2)); 
Robot.plot([sliderValue1*pi/180  sliderValue2*pi/180 sliderValue3*pi/180 0  0 0]); 
% sliderValue1 = handles.sliderValue1 ;
% sliderValue2 = handles.sliderValue2 ;
% sliderValue3 = handles.sliderValue3 ;
% sliderValue4 = handles.sliderValue4 ;
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(~, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Robot = handles.Robot
% handles.sliderValue1 = sliderValue1;
% handles.sliderValue2 = sliderValue2;
% handles.sliderValue3 = sliderValue3;
% handles.sliderValue4 = sliderValue4;
global sliderValue1 sliderValue2 sliderValue3 sliderValue4

sliderValue1 = get(handles.slider1,'Value');
sliderValue2 = get(handles.slider2,'Value');
sliderValue3 = get(handles.slider3,'Value');
set(handles.edit3,'String', num2str(sliderValue3)); 
Robot.plot([sliderValue1*pi/180  sliderValue2*pi/180 0 0  0 0]); 
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% sliderValue1 = handles.sliderValue1 ;
% sliderValue2 = handles.sliderValue2 ;
% sliderValue3 = handles.sliderValue3 ;
% sliderValue4 = handles.sliderValue4 ;

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Robot = handles.Robot
% handles.sliderValue1 = sliderValue1;
% handles.sliderValue2 = sliderValue2;
% handles.sliderValue3 = sliderValue3;
% handles.sliderValue4 = sliderValue4;
global sliderValue1 sliderValue2 sliderValue3 sliderValue4

sliderValue1 = get(handles.slider1,'Value');
sliderValue2 = get(handles.slider2,'Value');
sliderValue3 = get(handles.slider3,'Value');
sliderValue4 = get(handles.slider4,'Value');
set(handles.edit4,'String', num2str(sliderValue4)); 
Robot.plot([sliderValue1*pi/180  sliderValue2*pi/180 sliderValue3*pi/180 sliderValue4*pi/180 0 0]); 
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% sliderValue1 = handles.sliderValue1 ;
% sliderValue2 = handles.sliderValue2 ;
% sliderValue3 = handles.sliderValue3 ;
% sliderValue4 = handles.sliderValue4 ;


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% 
% % --- Executes on button press in pushbutton1.
% function pushbutton1_Callback(hObject, eventdata, handles)
% global target arr_sph arr_sph_c
% 
% % hObject    handle to pushbutton1 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% hold on
% global pos 
% global plott
% % pos = randi([5 20],1,3);
% x_0 = [];
% y_0 = [];
% if rem(plott,2)>=1
% y_0 = randi([5 18])
% x_0 = randi([-18 0])
% else 
%  x_0 = randi([5 18])
% y_0 = randi([-18 0])  
% end
% plott = 1 + plott;
% 
% [x,y,z] = sphere;
% r = 3;
% sphere_ = surf(x*r+x_0,y*r+y_0,z*r - 10+r) 
% % disp(target);
% pos = [x_0,y_0,-5];
% 
% arr_sph = [arr_sph,sphere_]
% arr_sph_c = [arr_sph,pos]

% % --- Executes on button press in pushbutton2.
% function pushbutton2_Callback(hObject, eventdata, handles)
% global pos 
%  global q_obj 
%  global q_t
%  global q_tpv
%  global qs1
%  Robot = handles.Robot
% 
% %disp(pos)
% tt = [0:0.1:2]
% T = 3; %time
% dt = 0.1;
% % qs1 = jtraj(q_obj,q_tpv,tt)
% % qs2 = jtraj(q_tpv,q_t,tt)
% % qs1_1 = calc_traj(q_obj(1),q_tpv(1),T,dt)'
% % qs1_2 = calc_traj(q_obj(2),q_tpv(2),T,dt)'
% % qs1_3 = calc_traj(q_obj(3),q_tpv(3),T,dt)'
% % qs1_4 = calc_traj(q_obj(4),q_tpv(4),T,dt)'
% %qs1 = [qs1_1 qs1_2 qs1_3 qs1_4]
% qs1 = jtraj(q_obj,q_tpv,tt);
% % qs2_1 = calc_traj(q_tpv(1),q_t(1),T,dt)'
% % qs2_2 = calc_traj(q_tpv(2),q_t(2),T,dt)'
% % qs2_3 = calc_traj(q_tpv(3),q_t(3),T,dt)'
% % qs2_4 = calc_traj(q_tpv(4),q_t(4),T,dt)'
% % qs2 = [qs2_1 qs2_2 qs2_3 qs2_4]
% % disp(qs1)
% % qs2 = calc_traj(q_tpv,q_t,T,dt)'
% TT1 = Robot.fkine(qs1);
% % TT2 = Robot.fkine(qs2);
% p_t1 = transl(TT1);
% % p_t2 = transl(TT2);
% hold on
% % plot3(p_t1(:,1),p_t1(:,2),p_t1(:,3))%,'-*','Color','b',3)
% plot3(p_t1(:,1),p_t1(:,2),p_t1(:,3),'Color','r','LineWidth',3) 
% % plot3(p_t2(:,1),p_t2(:,2),p_t2(:,3))%,'-*','Color','b',3)
% hold on
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% mcalc_traj(-pi/3,pi/2,-pi/7,-pi/2,pi/6,pi/9,-pi/5,pi,3,0.1,[0 2*pi],[-pi/4 pi/4],[-pi/4 pi/4],[-pi/4 pi/4])

% % --- Executes on button press in pushbutton3.
% function pushbutton3_Callback(hObject, eventdata, handles)
% global pos target 
% global q_obj
% global targets
% global q_t
% global q_tpv
% 
% Robot = handles.Robot
% 
% % hObject    handle to pushbutton3 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% m = [1 1 1 0 0 0]; %1X6
% %T_t = SE3(-8,4,0)%transl(pos)
% pos(3) = 0;
% T_obj = SE3(pos);%transl(pos)
% %q_obj = Robot.ikine(T_obj,[0 0 0 0],'mask',m);
%  q_obj = Robot.ikcon(T_obj);
% 
% disp(q_obj)
% target_pv = target;
% target_pv(3) = 5;
% T_tpv = SE3(target_pv)%transl(pos)
% %q_tpv = Robot.ikine(T_tpv,[0 0 0 0],'mask',m);
% q_tpv = Robot.ikcon(T_tpv);
% disp(target_pv);
% disp(q_tpv)
% % T_t = SE3(target)%transl(pos)
% % q_t = Robot.ikine(T_t,[0 0 0 0],'mask',m)
% % disp(q_t)

% 
% % --- Executes on button press in pushbutton4.
% function pushbutton4_Callback(hObject, eventdata, handles)
% global qs1
% Robot = handles.Robot
% 
% % hObject    handle to pushbutton4 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% hold on
% 
% for i=1:length(qs1)
%    Robot.plot([qs1(i,1) qs1(i,2) qs1(i,3)  qs1(i,4)   qs1(i,5)  qs1(i,6)]) ;
%    
% end
% % hold off
% 

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  target first_place  arr_sph arr_sph_c
Robot = handles.Robot
m = [1 1 1 0 0 0]; %1X6
tt = [0:0.1:2];
disp(length(arr_sph))
disp(length(arr_sph_c))
for ii=1:length(arr_sph_c)
set(handles.edit11,'String', strcat('Task',' ' , num2str(ii),'/',num2str(length(arr_sph_c)))); 

sphere_ = arr_sph(ii);
pos = [cell2mat(arr_sph_c(ii))]

pos_ = pos;
pos_(3) = 0;
pos_t = SE3(pos_);
target_pv = target;
target_pv(3) = 3;

target_pv_ = SE3(target_pv);
Ts0 = ctraj(SE3(first_place),pos_t,length(tt))
pp0 = Ts0.transl
plot3(pp0(:,1),pp0(:,2),pp0(:,3),'Color','b','LineWidth',1) 
hold on

for i=1:length(Ts0)
    zxc = Robot.ikine6s(Ts0(i),'ldf')
    Robot.plot(zxc) ;
    xyz_ = transl(Ts0(i))
    set(handles.edit5,'String', num2str(round(xyz_(1)))); 
    set(handles.edit6,'String', num2str(round(xyz_(2)))); 
    set(handles.edit7,'String', num2str(round(xyz_(3)))); 
    set(handles.edit8,'String', num2str(round(rad2deg(zxc(1))))); 
    set(handles.edit9,'String', num2str(round(rad2deg(zxc(2))))); 
    set(handles.edit10,'String', num2str(round(rad2deg(zxc(3))))); 
    
end

pos__ = pos;
pos__(3) = -5;
pos__t = SE3(pos__);

Ts00 = ctraj(pos_t,pos__t,10)
pp00 = Ts00.transl

plot3(pp00(:,1),pp00(:,2),pp00(:,3),'Color','b','LineWidth',1) 
hold on
for i=1:length(Ts00)
    zxc = Robot.ikine6s(Ts00(i),'ldf')
    Robot.plot(zxc) ;
    xyz_ = transl(Ts0(i))
    set(handles.edit5,'String',num2str(round(xyz_(1)))); 
    set(handles.edit6,'String',num2str(round(xyz_(2)))); 
    set(handles.edit7,'String', num2str(round(xyz_(3)))); 
    set(handles.edit8,'String', num2str(round(rad2deg(zxc(1))))); 
    set(handles.edit9,'String', num2str(round(rad2deg(zxc(2))))); 
    set(handles.edit10,'String', num2str(round(rad2deg(zxc(3))))); 
end

Ts00 = ctraj(pos__t,pos_t,10)
pp00 = Ts00.transl

plot3(pp00(:,1),pp00(:,2),pp00(:,3),'Color','b','LineWidth',1) 
hold on
[x,y,z] = sphere;
r = 3;
   
for i=1:length(Ts00)
    zxc = Robot.ikine6s(Ts00(i),'ldf')
    Robot.plot(zxc) ;
    delete (sphere_)
    xyz = transl(Ts00(i))
    sphere_ = surf(x*r+xyz(1),y*r+xyz(2),z*r +xyz(3)) 
    xyz_ = transl(Ts00(i))
    set(handles.edit5,'String', num2str(round(xyz_(1)))); 
    set(handles.edit6,'String', num2str(round(xyz_(2)))); 
    set(handles.edit7,'String',num2str(round(xyz_(3)))); 
    set(handles.edit8,'String', num2str(round(rad2deg(zxc(1))))); 
    set(handles.edit9,'String', num2str(round(rad2deg(zxc(2))))); 
    set(handles.edit10,'String', num2str(round(rad2deg(zxc(3))))); 
end

Ts = ctraj(pos_t,target_pv_,length(tt))
pp = Ts.transl
hold on
plot3(pp(:,1),pp(:,2),pp(:,3),'Color','b','LineWidth',1) 
for i=1:length(Ts)
    zxc = Robot.ikine6s(Ts(i),'ldf')
    xyz = transl(Ts(i))
    Robot.plot(zxc) ;
    delete (sphere_)
    sphere_ = surf(x*r+xyz(1),y*r+xyz(2),z*r +xyz(3)) 
    xyz_ = transl(Ts(i))
    set(handles.edit5,'String', num2str(round(xyz_(1)))); 
    set(handles.edit6,'String', num2str(round(xyz_(2)))); 
    set(handles.edit7,'String', num2str(round(xyz_(3)))); 
    set(handles.edit8,'String', num2str(round(rad2deg(zxc(1))))); 
    set(handles.edit9,'String', num2str(round(rad2deg(zxc(2))))); 
    set(handles.edit10,'String', num2str(round(rad2deg(zxc(3))))); 
end

Ts = ctraj(target_pv_,SE3(target),10)
pp = Ts.transl
hold on
plot3(pp(:,1),pp(:,2),pp(:,3),'Color','b','LineWidth',2) 
for i=1:length(Ts)
    zxc = Robot.ikine6s(Ts(i),'ldf')
    xyz = transl(Ts(i))
    Robot.plot(zxc) ;
    delete (sphere_)
    sphere_ = surf(x*r+xyz(1),y*r+xyz(2),z*r +xyz(3)) 
    xyz_ = transl(Ts(i))
    set(handles.edit5,'String', num2str(round(xyz_(1)))); 
    set(handles.edit6,'String', num2str(round(xyz_(2)))); 
    set(handles.edit7,'String', num2str(round(xyz_(3)))); 
    set(handles.edit8,'String', num2str(round(rad2deg(zxc(1))))); 
    set(handles.edit9,'String', num2str(round(rad2deg(zxc(2))))); 
    set(handles.edit10,'String', num2str(round(rad2deg(zxc(3))))); 
end
delete(sphere_)
sphere_ = surf(x*r+target(1),y*r+target(2),z*r +target(3)) 

Ts = ctraj(SE3(target),target_pv_,10)
pp = Ts.transl
hold on
plot3(pp(:,1),pp(:,2),pp(:,3),'Color','b','LineWidth',1) 
for i=1:length(Ts)
    zxc = Robot.ikine6s(Ts(i),'ldf')
    xyz = transl(Ts(i))
    Robot.plot(zxc) ;
    xyz_ = transl(Ts(i))
    set(handles.edit5,'String', num2str(round(xyz_(1)))); 
    set(handles.edit6,'String', num2str(round(xyz_(2)))); 
    set(handles.edit7,'String', num2str(round(xyz_(3)))); 
    set(handles.edit8,'String', num2str(round(rad2deg(zxc(1))))); 
    set(handles.edit9,'String', num2str(round(rad2deg(zxc(2))))); 
    set(handles.edit10,'String', num2str(round(rad2deg(zxc(3))))); 
    
end
first_place = target_pv

end
set(handles.edit11,'String','ALL DONE'); 
 
% % --- Executes on button press in pushbutton6.
% function pushbutton6_Callback(hObject, eventdata, handles)
% % hObject    handle to pushbutton6 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% global pos target 
% global q_obj
% global targets
% global q_t
% global q_tpv
% 
% Robot = handles.Robot
% 
% % hObject    handle to pushbutton3 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% m = [1 1 1 0 0 0]; %1X6
% %T_t = SE3(-8,4,0)%transl(pos)
% pos(3) = 0
% T_obj = SE3(pos);%transl(pos)
% % q_obj = Robot.ikine6s(T_obj,'lun')%,[0 0 0 0],'mask',m);
% %  q_obj = Robot.ikcon(T_obj);
%     q_obj = Robot.ikine6s(T_obj,'ldf')%,'mask',m)
% 
% disp(q_obj)
% target_pv = target;
% target_pv(3) = 0;
% T_tpv = SE3(target_pv)%transl(pos)
% q_tpv = Robot.ikine6s(T_tpv,'ldf');
% %q_tpv = Robot.ikcon(T_tpv);
% disp(target_pv)
% disp(q_tpv)
% % T_t = SE3(target)%transl(pos)
% % q_t = Robot.ikine(T_t,[0 0 0 0],'mask',m)
% % disp(q_t)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global target
global target arr_sph arr_sph_c

% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hold on
global  sphere_ arr_sph
global plott
% pos = randi([5 20],1,3);
x_0 = [];
y_0 = [];
if rem(plott,2)>=1
y_0 = randi([5 18])
x_0 = randi([-18 0])
else 
 x_0 = randi([5 18])
y_0 = randi([-18 0])  
end
plott = 1 + plott;
[x,y,z] = sphere;
r = 3;
sphere_ = surf(x*r+x_0,y*r+y_0,z*r - 10+r) 
% disp(target);
pos = [x_0,y_0,-5];
arr_sph = [arr_sph,sphere_]
arr_sph_c = [arr_sph_c,pos]
set(handles.edit11,'String', strcat('Task',' ' , num2str(0),'/',num2str(length(arr_sph_c)))); 

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
global arr_sph
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(arr_sph(1))



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
