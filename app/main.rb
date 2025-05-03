# Boid algorithm implementation

class Boid
  attr_accessor :x, :y, :vx, :vy, :dna

  def initialize(x, y, dna = nil)
    @x, @y = x, y
    # randomly pick a starting angle
    angle = rand * Math::PI * 2
    # use cos, sin to find the individual edges (aka velocities) for the x and y components of the angle (direction)
    @vx, @vy = Math.cos(angle) * 2, Math.sin(angle) * 2

    # DNA hash
    if dna.nil?
      @dna = {
        birth: Kernel.tick_count,
        alignment_weight: Numeric.rand(0.5..2.0),
        cohesion_weight: Numeric.rand(0.5..2.0),
        separation_weight: Numeric.rand(1.5..3.0),
        perception_radius: Numeric.rand(40..100),
        max_speed: Numeric.rand(2.0..5.0),
        # size_w_h: Numeric.rand(8.0..12.0),
        # size_w_h: Numeric.rand(16.0..32.0),
        size_w: 24,
        size_h: 18,
        vis_red: 255,
        vis_green: 255,
        vis_blue: 255,
        expired: false
      }
    else
      @dna = dna
    end
  end

  def update(args, boids)
    # each boid will only be "conscious" of other boids within 50px
    perception_radius = dna.perception_radius

    # temp hashes for each part of the algoritm:
    # alignment (velocity), cohesion (direction), separation (social distancing)
    alignment = { x: 0, y: 0 }
    cohesion = { x: 0, y: 0 }
    separation = { x: 0, y: 0 }
    # keep a count of the total number of other boids perceived and processed
    total = 0
    # steer forces
    steer_x = 0.0
    steer_y = 0.0

    boids.each do |other|
      # find the distance (hypotenuse) between self and other boid
      # skip if the current boid is self or outside of the perception radius
      next if other == self
      dist = Math.hypot(other.x - x, other.y - y)
      next if dist > perception_radius

      # Alignment - sum the x and y velocities of each perceived boid
      alignment.x += other.vx
      alignment.y += other.vy

      # Cohesion - sum the x and y positions of each perceived boid
      cohesion.x += other.x
      cohesion.y += other.y

      # Separation - try not to bump into other boids
      if dist < 25
        # guard against divide by zero cases
        factor = 1.0 / (dist + 0.01)
        separation.x += (x - other.x) * factor
        separation.y += (y - other.y) * factor
      end

      total += 1
    end

    if total > 0
      # Alignment
      alignment.x /= total
      alignment.y /= total
      alignment_mag = Math.hypot(alignment.x, alignment.y)
      # normalize and scale the magnitude for smooth motion
      if alignment_mag > 0
        alignment = Geometry.vec2_normalize(alignment)
      end
      # scale strength by alighnment weight
      alignment.x *= dna.alignment_weight
      alignment.y *= dna.alignment_weight

      # Cohesion
      # avg the group's position (center of mass)
      # subtract current boid's position to get the vector to the group
      # scale by 0.05 to adjust gently then apply the dna encoded cohesion weight
      cohesion.x = ((cohesion.x / total) - x) * 0.05
      cohesion.y = ((cohesion.y / total) - y) * 0.05
      cohesion.x *= dna.cohesion_weight
      cohesion.y *= dna.cohesion_weight

      # Separation
      separation.x *= dna.separation_weight
      separation.y *= dna.separation_weight

      # Combine steering forces
      steer_x += alignment.x + cohesion.x + separation.x
      steer_y += alignment.y + cohesion.y + separation.y
    end

    # Limit steering force
    steer_mag = Math.hypot(steer_x, steer_y)
    max_force = 0.5

    if steer_mag > max_force
      steer_x = (steer_x / steer_mag) * max_force
      steer_y = (steer_y / steer_mag) * max_force
    end

    # apply steering force to velocities
    @vx += steer_x
    @vy += steer_y

    # Limit speed
    speed = Math.hypot(@vx, @vy)
    min_speed = 0.5
    max_speed = dna.max_speed

    if speed > max_speed
      @vx = (@vx / speed) * max_speed
      @vy = (@vy / speed) * max_speed
    elsif speed < min_speed
      @vx = (@vx / speed) * min_speed
      @vy = (@vy / speed) * min_speed
    end

    # update the boid's position with the new vectors
    @x += @vx
    @y += @vy

    # Wrap around screen
    @x %= Grid.w
    @y %= Grid.h

    # TODO This is not a good lifespan simulation LOL
    args.state.boids.shift if args.state.boids.count > 200

    # DEBUG
    # if args.state.boids.count > 200
      # lucky = args.state.boids.shift
      # show_message("#{lucky} has expired")
    # end
    # END DEBUG

  end
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
  args.state.gen_counter ||= 1
  args.outputs.background_color = { r: 30, g: 30, b: 30, a: 255 }
  
  # initialize
  args.state.boids ||= Array.new(200) { Boid.new(rand(Grid.w), rand(Grid.h)) }

  # create & mutate every 5 seconds
  if  Kernel.tick_count.zmod?(300) && !Kernel.tick_count.zero?
    args.state.gen_counter += 1
    parent = args.state.boids.sample

    args.state.boids << Boid.new(parent.x, parent.y, mutate(parent.dna))

    # DEBUG
    # show_message("#{args.state.boids.last} size: #{args.state.boids.last.dna.size_w_h}")
    # show_message("#{args.state.boids.last} color: #{args.state.boids.last.dna.vis_red}, #{args.state.boids.last.dna.vis_green}, #{args.state.boids.last.dna.vis_blue}")
    # END DEBUG
  end

  # update
  args.state.boids.each { |boid| boid.update(args, args.state.boids) }

  # destroy dead boids
  args.state.boids.reject! { |b| b.dna.expired }

  # draw
  draw_boids(args)

  # DEBUG
  # args.outputs.primitives << args.gtk.framerate_diagnostics_primitives
  args.outputs.debug << "#{args.gtk.current_framerate.to_sf}"
  args.outputs.debug << "generations: #{args.state.gen_counter}"
  args.outputs.debug << "boid count: #{args.state.boids.length}"
  # END DEBUG
end

def show_message(msg)
  cur_time = Time.now
  p "#{cur_time.to_s.split[1]}: #{msg}"
end
