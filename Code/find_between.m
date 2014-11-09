function ret_index = find_between(obstacle, index, next_index, mx, my)
start_x = obstacle.X(index);
start_y = obstacle.Y(index);

end_x = obstacle.X(next_index);
end_y = obstacle.Y(next_index);

if end_x < start_x
    end_x = obstacle.X(index);
    start_x = obstacle.X(next_index);
end

if end_y < start_y
    end_y = obstacle.Y(index);
    start_y = obstacle.Y(next_index);
end    
    
ret_index = -1;

for i = 1:numel(mx)   
    
    if(mx(i) >= start_x && mx(i) <= end_x && my(i) >= start_y  && my(i) <=end_y)
        ret_index = i
        input(' ');
        break;
    end
end

