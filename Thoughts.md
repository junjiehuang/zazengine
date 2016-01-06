when storing linear-depth then one must calculate linear depth in other shaders with utilize the depth-buffer for correct depth-visibility too.

problem with setting gl\_FragDepth: overrides early z-culling

double speed for z-rendering:
- color writes disabled
- alpha-test disabled
- framgent shader discards or writes depth

conditional rendering: send down light-volume (should be extremly fast) and do conditional render on the light.

instancing: the goal is to render 100 of the teapots