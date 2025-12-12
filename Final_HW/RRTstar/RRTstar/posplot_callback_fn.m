function posplot_callback_fn(obj,eventdata,app)
    global position
    global instant_pos
    global commandposition
    if ~isempty(instant_pos)
        plot(app.UIAxes_realtimeresult,instant_pos(1),instant_pos(2),'s','Color','r','MarkerSize',12);
        hold(app.UIAxes_realtimeresult,'on')
        if ~isempty(commandposition)
            plot(app.UIAxes_realtimeresult,commandposition(1),commandposition(2),'s','Color','b','MarkerSize',12);
            hold(app.UIAxes_realtimeresult,'on')
        end
        
        app.PoseEditField.Value=double(instant_pos(3));
    end
    
end