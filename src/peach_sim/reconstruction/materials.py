"""New Blender materials: paper fibre, bark, and translucent peach leaves."""
import bpy


def material(name, color, roughness):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*color, 1)
    m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = (*color, 1)
    p.inputs['Roughness'].default_value = roughness
    p.inputs['Specular IOR Level'].default_value = .18
    return m, p


def textured(name, colors, scale, bump_distance, roughness=.6):
    m, p = material(name, colors[0], roughness)
    n = m.node_tree.nodes
    l = m.node_tree.links
    tex = n.new('ShaderNodeTexNoise')
    tex.inputs['Scale'].default_value = scale
    tex.inputs['Detail'].default_value = 3
    ramp = n.new('ShaderNodeValToRGB')
    ramp.color_ramp.elements[0].position = .18
    ramp.color_ramp.elements[1].position = .82
    for e, c in zip(ramp.color_ramp.elements, colors):
        e.color = (*c, 1)
    l.new(tex.outputs['Fac'], ramp.inputs[0])
    l.new(ramp.outputs['Color'], p.inputs['Base Color'])
    fine = n.new('ShaderNodeTexNoise')
    fine.inputs['Scale'].default_value = scale * 35
    fine.inputs['Detail'].default_value = 2
    bump = n.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = .3
    bump.inputs['Distance'].default_value = bump_distance
    l.new(fine.outputs['Fac'], bump.inputs['Height'])
    l.new(bump.outputs['Normal'], p.inputs['Normal'])
    if 'Bark' in name:
        coords = n.new('ShaderNodeTexCoord')
        mapping = n.new('ShaderNodeVectorMath')
        mapping.operation = 'MULTIPLY'
        mapping.inputs[1].default_value = (6, 6, .65)
        l.new(coords.outputs['Generated'], mapping.inputs[0])
        cells = n.new('ShaderNodeTexVoronoi')
        cells.feature = 'DISTANCE_TO_EDGE'
        cells.inputs['Scale'].default_value = 24
        l.new(mapping.outputs[0], cells.inputs['Vector'])
        cracks = n.new('ShaderNodeBump')
        cracks.inputs['Distance'].default_value = .004
        cracks.inputs['Strength'].default_value = .8
        l.new(cells.outputs['Distance'], cracks.inputs['Height'])
        l.new(bump.outputs['Normal'], cracks.inputs['Normal'])
        l.new(cracks.outputs[0], p.inputs['Normal'])
    return m


def create():
    mats = {}
    for i, (a, b) in enumerate([
        ((.18, .022, .025), (.32, .058, .047)),
        ((.25, .033, .025), (.39, .075, .047)),
        ((.19, .030, .025), (.32, .065, .047)),
        ((.36, .195, .072), (.53, .31, .13)),
    ]):
        mats[f'paper{i}'] = textured(
            f'Paper / pigment variant {i}', (a, b), 8, .00025, .74)
    mats['bark'] = textured('Bark / aged silver brown',
                            ((.022, .018, .013), (.13, .105, .075)), 18, .012, .86)
    mats['twig'] = textured(
        'One year fruiting wood', ((.07, .032, .018), (.18, .105, .055)), 9, .0006, .52)
    mats['wire'] = material('Twisted matte wire', (.13, .11, .07), .45)[0]
    mats['fruit'] = textured(
        'Peach / enclosed fruit', ((.55, .19, .043), (.85, .47, .16)), 9, .00015, .65)
    for i in range(5):
        m, p = material(
            f'Leaf / variant {i}', (.03 + i * .007, .09 + i * .016, .012 + i * .003), .4)
        n = m.node_tree.nodes
        l = m.node_tree.links
        uv = n.new('ShaderNodeTexCoord')
        sep = n.new('ShaderNodeSeparateXYZ')
        l.new(uv.outputs['UV'], sep.inputs[0])
        # Cross-leaf green variation, midrib, and repeating angled secondary
        # veins.
        ramp = n.new('ShaderNodeValToRGB')
        r = ramp.color_ramp
        r.elements[0].position = .0
        r.elements[0].color = (.009, .036, .004, 1)
        r.elements[1].position = 1.
        r.elements[1].color = (.011, .042, .004, 1)
        for pos, c in [(.43, (.022 + i * .003, .072 + i * .006, .012, 1)), (.493,
                                                                            (.07, .12, .024, 1)), (.507, (.07, .12, .024, 1)), (.57, (.030, .082, .016, 1))]:
            r.elements.new(pos).color = c
        l.new(sep.outputs['Y'], ramp.inputs[0])
        l.new(ramp.outputs[0], p.inputs['Base Color'])
        # Repeated oblique secondary veins, mirrored on both sides of the
        # midrib.

        def mathnode(op, a, b=None):
            q = n.new('ShaderNodeMath')
            q.operation = op
            if isinstance(a, (float, int)):
                q.inputs[0].default_value = a
            else:
                l.new(a, q.inputs[0])
            if b is not None:
                if isinstance(b, (float, int)):
                    q.inputs[1].default_value = b
                else:
                    l.new(b, q.inputs[1])
            return q.outputs[0]
        across = mathnode(
            'ABSOLUTE', mathnode(
                'SUBTRACT', sep.outputs['Y'], .5))
        phase = mathnode(
            'SUBTRACT', mathnode(
                'MULTIPLY', sep.outputs['X'], 15), mathnode(
                'MULTIPLY', across, 3.5))
        vein = mathnode('LESS_THAN', mathnode('FRACT', phase), .075)
        bump = n.new('ShaderNodeBump')
        bump.inputs['Distance'].default_value = .00015
        bump.inputs['Strength'].default_value = .55
        l.new(vein, bump.inputs['Height'])
        l.new(bump.outputs[0], p.inputs['Normal'])
        p.inputs['Subsurface Weight'].default_value = .045
        p.inputs['Roughness'].default_value = .48 + i * .035
        p.inputs['Specular IOR Level'].default_value = .22
        trans = n.new('ShaderNodeBsdfTranslucent')
        l.new(ramp.outputs[0], trans.inputs[0])
        mix = n.new('ShaderNodeMixShader')
        mix.inputs[0].default_value = .22
        l.new(p.outputs[0], mix.inputs[1])
        l.new(trans.outputs[0], mix.inputs[2])
        l.new(mix.outputs[0], n.get('Material Output').inputs['Surface'])
        mats[f'leaf{i}'] = m
    mats['soil'] = textured('Soil / humus and dry crumbs',
                            ((.035, .022, .012), (.16, .105, .055)), 45, .012, .94)
    mats['grass'] = textured('Grass / living groundcover',
                             ((.045, .08, .009), (.12, .19, .026)), 8, .001, .66)
    return mats
