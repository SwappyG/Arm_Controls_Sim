function matrix = roty(phi)
    % phi must be in radians
    matrix = [cos(phi)      0    sin(phi)    ;   ...
              0              1     0           ;   ...
              -sin(phi)     0    cos(phi)    ]   ;
end