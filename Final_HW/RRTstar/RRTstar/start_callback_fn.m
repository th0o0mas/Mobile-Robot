function start_callback_fn(obj,eventdata,app)
    global position
    position=[0;0;0];
    cla(app.UIAxes_realtimeresult);
    cla(app.UIAxes_trajectory);

    %overlapping for analysis
    ax3=findobj(app.UIAxes_pathresult,'Type','axes');
    cla(app.UIAxes_trajectory);
    copyobj(allchild(ax3),app.UIAxes_realtimeresult);
    hold(app.UIAxes_realtimeresult,'on');
end