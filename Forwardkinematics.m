function Forwardkinematics(j)
global ulink

if j == 0 return; end
if j ~= 1
    i = ulink(j).mother;
    ulink(j).p = ulink(i).R * ulink(j).b + ulink(i).p;
    ulink(j).R = ulink(i).R * Rodrigues(ulink(j).a, ulink(j).q);
end
Forwardkinematics(ulink(j).sister);
Forwardkinematics(ulink(j).child);