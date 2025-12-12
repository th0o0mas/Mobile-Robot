t=timer(StartDelay=1,Period=0.05,ExecutionMode="fixedRate",TimerFcn=@Fun,StartFcn=@SFun);






start(t);
pause(30)
stop(t)
function Fun(~,~)
    toc()
    pause(0.04)
end
function SFun(~,~)

    tic()
end