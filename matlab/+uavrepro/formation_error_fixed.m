function e_i = formation_error_fixed(i, positions, desired, leader_position, A, B)
%FORMATION_ERROR_FIXED Fixed-topology formation error from Eq. (12).

N = size(positions, 1);
e_i = zeros(2, 1);

for j = 1:N
    e_i = e_i + A(i, j) * ((positions(i, :) - desired(i, :)) - (positions(j, :) - desired(j, :))).';
end

e_i = e_i + B(i) * (positions(i, :) - desired(i, :) - leader_position.').';
end
