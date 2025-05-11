# Boid algorithm implementation
require 'app/boid.rb'
require 'app/quadtree.rb'

# Constants
MAX_BOIDS = 300
BASE_FLOCK_SIZE = 300

def tick(args)
  ### initializations
  args.state.show_debug ||= false
  # bg
  args.outputs.background_color = { r: 30, g: 30, b: 30, a: 255 }
  # generation counter
  args.state.gen_counter ||= 1
  # quadtree
  args.state.quadtree ||= Quadtree.new({ x: 0, y: 0, w: Grid.w, h: Grid.h }, 4)
  # clear the quadtree to be repopulated each tick
  args.state.quadtree.clear
  # initialize the core flock
  args.state.boids ||= Array.new(BASE_FLOCK_SIZE) { Boid.new(rand(Grid.w), rand(Grid.h)) }

  ### main loop
  # insert boids into quadtree
  args.state.boids.each { |boid| args.state.quadtree.insert({ x: boid.x, y: boid.y, data: boid }) }

  # create a new boid & mutate every 5 seconds
  if Kernel.tick_count.zmod?(300) && !Kernel.tick_count.zero?
    args.state.gen_counter += 1
    parent = args.state.boids.sample

    args.state.boids << Boid.new(parent.x, parent.y, mutate(parent.dna))
  end

  # update boids
  args.state.boids.each do |boid|
    radius= boid.dna.perception_radius
    nearby_boids = args.state.quadtree.query({
                                               x: boid.x - radius,
                                               y: boid.y - radius,
                                               w: radius * 2,
                                               h: radius * 2
                                             })
    boid.update(args, nearby_boids.map { |b| b.data }) # Pass only nearby boids
  end

  # destroy dead boids
  args.state.boids.shift if args.state.boids.count > MAX_BOIDS

  # draw
  render_boids(args)

  # DEBUG
  args.state.show_debug = !args.state.show_debug if args.inputs.keyboard.key_down.zero

  if args.state.show_debug
    # args.outputs.primitives << args.gtk.framerate_diagnostics_primitives
    args.outputs.debug << "#{args.gtk.current_framerate.to_sf}"
    args.outputs.debug << "generations: #{args.state.gen_counter}"
    args.outputs.debug << "boids: #{args.state.boids.length}"
    args.state.quadtree.render_quadtree(args)
    args.outputs[:rt_master].primitives << {
      x: 0,
      y: 0,
      w: Grid.w,
      h: Grid.h,
      path: :rt_quadtree
    }.sprite!
  end
  # END DEBUG

  args.outputs.primitives << {
    x: 0,
    y: 0,
    w: Grid.w,
    h: Grid.h,
    path: :rt_master
  }.sprite!
end

def mutate(src_dna)
  # transform the inheritde dna data
  mutated = src_dna.transform_values.with_index do |val, i|
    case src_dna.keys[i]
    when :alignment_weight then val + Numeric.rand(-1.0..1.0)
    when :cohesion_weight then val + Numeric.rand(-1.0..1.0)
    when :separation_weight then val + Numeric.rand(-1.0..1.0)
    when :perception_radius then val + Numeric.rand(-1.0..1.0)
    when :max_speed then val + Numeric.rand(-1.0..1.0)
    when :size_w then val + Numeric.rand(-3.0..3.0)
    when :size_h then val + Numeric.rand(-3.0..3.0)
    when :vis_red then val + Numeric.rand(-50.0..25.0)
    when :vis_green then val + Numeric.rand(-50.0..25.0)
    when :vis_blue then val + Numeric.rand(-50.0..25.0)
    else val
    end
  end

  # clamp values within acceptable ranges
  mutated.alignment_weight = mutated.alignment_weight.clamp(0.1, 3.0)
  mutated.cohesion_weight = mutated.cohesion_weight.clamp(0.1, 3.0)
  mutated.separation_weight = mutated.separation_weight.clamp(0.1, 3.0)
  mutated.perception_radius = mutated.perception_radius.clamp(20, 150)
  mutated.max_speed = mutated.max_speed.clamp(1.0, 6.0)
  mutated.size_w = mutated.size_w.clamp(6.0, 256.0)
  mutated.size_h = mutated.size_h.clamp(6.0, 256.0)
  mutated.vis_red = mutated.vis_red.clamp(0.0, 255.0)
  mutated.vis_green = mutated.vis_green.clamp(0.0, 255.0)
  mutated.vis_blue = mutated.vis_blue.clamp(0.0, 255.0)

  return mutated
end

def render_boids(args)
  args.outputs[:rt_master].primitives << args.state.boids.map do |b|
    {
      x: b.x - (0.5 * b.dna.size_w),
      y: b.y - (0.5 * b.dna.size_h),
      w: b.dna.size_w,
      h: b.dna.size_h,
      r: b.dna.vis_red,
      g: b.dna.vis_green,
      b: b.dna.vis_blue,
      a: 255,
      blendmode_enum: 1, # 1 --> default blending
      angle: Math.atan2(b.vy, b.vx).to_degrees,
      path: 'sprites/isometric/white.png'
    }.sprite!
  end
end

def show_message(msg)
  cur_time = Time.now
  p "#{cur_time.to_s.split[1]}: #{msg}"
end
