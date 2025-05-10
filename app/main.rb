# Boid algorithm implementation
require 'app/boid.rb'
require 'app/quadtree.rb'

# Constants
MAX_BOIDS = 315

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

def draw_boids(args)
  args.outputs.sprites << args.state.boids.map do |b|
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
    }
  end
end

def tick(args)
  # bg
  args.outputs.background_color = { r: 30, g: 30, b: 30, a: 255 }
  # boids things
  args.state.gen_counter ||= 1
  # quadtree
  args.state.quadtree ||= Quadtree.new({ x: 0, y: 0, w: Grid.w, h: Grid.h }, 4)
  args.state.quadtree.clear # clear the quadtree to be repopulated each tick
  
  # initialize the core flock
  args.state.boids ||= Array.new(MAX_BOIDS) { Boid.new(rand(Grid.w), rand(Grid.h)) }

  # insert boids into quadtree
  args.state.boids.each { |boid| args.state.quadtree.insert({ x: boid.x, y: boid.y, data: boid }) }

  # create a new boid & mutate every 5 seconds
  if Kernel.tick_count.zmod?(300) && !Kernel.tick_count.zero?
    args.state.gen_counter += 1
    parent = args.state.boids.sample

    args.state.boids << Boid.new(parent.x, parent.y, mutate(parent.dna))
  end

  # update
  # n^2 update algorithm
  # args.state.boids.each { |boid| boid.update(args, args.state.boids) }
  # quadtree update algorithm
  args.state.boids.each do |boid|
    nearby_boids = args.state.quadtree.query({
                                               x: boid.x - boid.dna.perception_radius,
                                               y: boid.y - boid.dna.perception_radius,
                                               w: 100,
                                               h: 100
                                             })
    boid.update(args, nearby_boids.map { |b| b.data }) # Pass only nearby boids
  end

  # destroy dead boids
  args.state.boids.reject! { |b| b.dna.expired }

  # draw
  draw_boids(args)

  # DEBUG
  # args.outputs.primitives << args.gtk.framerate_diagnostics_primitives
  args.outputs.debug << "#{args.gtk.current_framerate.to_sf}"
  args.outputs.debug << "generations: #{args.state.gen_counter}"
  args.outputs.debug << "boids: #{args.state.boids.length}"
  # END DEBUG
end

def show_message(msg)
  cur_time = Time.now
  p "#{cur_time.to_s.split[1]}: #{msg}"
end
