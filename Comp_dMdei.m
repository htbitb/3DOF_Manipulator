function dMdei = Comp_dMdei( M,ei )
dMdei=simplify([diff(M(1,1),ei),diff(M(1,2),ei),diff(M(1,3),ei);...
                diff(M(2,1),ei),diff(M(2,2),ei),diff(M(2,3),ei);...
                diff(M(3,1),ei),diff(M(3,2),ei),diff(M(3,3),ei)]);
end