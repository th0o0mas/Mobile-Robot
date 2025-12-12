%GO button pressed!
function GoRobotICP(app)
    global scanresult
    global position
    global commandposition
    global LidarCoord
    if ~exist("target_position.mat")
        disp("calculate again");
    else
        load("target_position.mat");
    end
    %rosinit()
    tposNode=ros2node("/GUI");
    pause(3);
    pool = parpool('Processes'); % run the scanMatch function in background
    f=parfeval(pool,@scanMatch,1,3);


    
    %publish test
    % targetposPub=ros2publisher(tposNode,"/target_position","geometry_msgs/Point");
    % PubMsg=ros2message(targetposPub);
    % t=timer('StartDelay',1,'Period',0.05,'ExecutionMode','fixedRate')
    % t.TimerFcn={@Publish_callback_fn,targetposPub,PubMsg,targetposition};
    
    t_plotpos=timer('StartDelay',0.5,'Period',0.1,'ExecutionMode','fixedRate')
    t_plotpos.StartFcn={@start_callback_fn,app};
    t_plotpos.TimerFcn={@posplot_callback_fn,app};
    targetposSub=ros2subscriber(tposNode,"/position","geometry_msgs/Point",@positionCallback);
    lidarscanSub=ros2subscriber(tposNode,"/scan","sensor_msgs/LaserScan",@LaserCallback);
    
    start(t_plotpos);
   



    
   
    ans=fetchOutputs(f);
   




    hold(app.UIAxes_trajectory,'off')
    clear Publish_callback_fn
    save('robotpath.mat',"position")
    stop(t_plotpos);
    delete(t_plotpos);
    % save("scanresult.mat","scanresult")
end