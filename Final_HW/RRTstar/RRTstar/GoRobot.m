%GO button pressed!
function GoRobot(app)
    global scanresult
    global position
    global commandposition
    global instant_pos
    instant_pos=[0 0 0];
    if ~exist("target_position.mat")
        disp("calculate again");
    else
        load("target_position.mat");
    end
    %rosinit()
    tposNode=ros2node("/GUI");
    pause(3);
    %publish test
    targetposPub=ros2publisher(tposNode,"/target_position","geometry_msgs/Point");
    PubMsg=ros2message(targetposPub);
    %t=timer('StartDelay',0.5,'Period',0.05,'ExecutionMode','fixedDelay')
    %t.TimerFcn={@Publish_callback_fn,targetposPub,PubMsg,targetposition};
    t_plotpos=timer('StartDelay',0.05,'Period',0.05,'ExecutionMode','fixedRate')
    t_plotpos.StartFcn={@start_callback_fn,app};
    t_plotpos.TimerFcn={@posplot_callback_fn,app};
    targetposSub=ros2subscriber(tposNode,"/position","geometry_msgs/Point",@positionCallback);
    lidarscanSub=ros2subscriber(tposNode,"/scan","sensor_msgs/LaserScan",@LaserCallback);
    app.StatusTextArea.Value="Running";
    start(t_plotpos);
    %start(t);
    %End time
    EndTime=30;
    ErrorDis=5;
    tic();
    rate=rateControl(20);
    tposidx=1;
    while  tposidx<=size(targetposition,1)
        %if running time exceeds limit, break the loop
        instant_pos
         t_EndTime=toc();
         if t_EndTime>EndTime
             break;
         end
         %Calculate distance Error between target and current position
         temp_dis=[targetposition(tposidx,1)-instant_pos(1);targetposition(tposidx,2)-instant_pos(2);0];
         dis_err=sqrt(temp_dis'*temp_dis);
         
         if (tposidx < size(targetposition,1)) && (dis_err<ErrorDis)
            
            tposidx=tposidx+1;
            PubMsg.x=targetposition(tposidx,1)*0.01;
            PubMsg.y=targetposition(tposidx,2)*0.01;
            PubMsg.z=0;
            %Publish only when robot reaches close enough
            send(targetposPub,PubMsg);
            
        elseif tposidx >= size(targetposition,1) && (dis_err<ErrorDis)
            disp("break")
            break;
        end
        commandposition(1)=targetposition(tposidx,1);
        commandposition(2)=targetposition(tposidx,2);
        waitfor(rate);
    end


    %pause(30);
    %stop(t)
    %delete(t)
    stop(t_plotpos);
    %delete(t_plotpos);
    hold(app.UIAxes_realtimeresult,'off')
    hold(app.UIAxes_trajectory,'off')
    clear Publish_callback_fn
    %rosshutdown()
    save('robotpath.mat',"position")
    save("scanresult.mat","scanresult")
end