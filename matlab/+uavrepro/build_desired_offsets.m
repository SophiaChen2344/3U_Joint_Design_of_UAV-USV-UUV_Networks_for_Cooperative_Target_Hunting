function desired = build_desired_offsets(radius, num_followers)
%BUILD_DESIRED_OFFSETS Desired pentagon-style formation offsets from Section IV.

desired = zeros(num_followers, 2);
for i = 1:num_followers
    angle = 2 * pi * i / num_followers;
    desired(i, :) = [radius * cos(angle), radius * sin(angle)];
end
end
