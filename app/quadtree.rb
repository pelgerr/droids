class Quadtree
  attr_reader :bounds, :capacity, :objects, :divided

  def initialize(bounds, capacity)
    @bounds = bounds # bounds is a hash with :x, :y, :w, :h
    @capacity = capacity # maximum objects before subdivision
    @objects = []
    @divided = false
  end

  def insert(object)
    # Early exit if the object is outside the bounds
    return false unless contains?(@bounds, object)

    if @objects.length < @capacity
      @objects << object
      return true
    else
      subdivide unless @divided
      return (@northwest.insert(object) || @northeast.insert(object) ||
              @southwest.insert(object) || @southeast.insert(object))
    end
  end

  def query(range, found = [])
    # Return early if bounds do not intersect
    return found unless intersects?(@bounds, range)

    # Add any objects within the range
    found.concat(@objects.select { |object| contains?(range, object) })

    # Query child nodes if subdivided
    if @divided
      @northwest.query(range, found)
      @northeast.query(range, found)
      @southwest.query(range, found)
      @southeast.query(range, found)
    end

    found
  end

  def clear
    # Clear the quadtree and its subdivisions
    @objects = []
    @divided = false
    @northwest = @northeast = @southwest = @southeast = nil
  end

  def render_quadtree(args)
    # Visualize the quadtree for debugging
    args.outputs[:rt_quadtree].primitives << {
      x: @bounds.x,
      y: @bounds.y,
      w: @bounds.w,
      h: @bounds.h,
      r: 255,
      g: 0,
      b: 0,
      a: 255
    }.border!

    if @divided
      @northwest.render_quadtree(args)
      @northeast.render_quadtree(args)
      @southwest.render_quadtree(args)
      @southeast.render_quadtree(args)
    end
  end

  
  private

  # Check if a point lies within a rectangle
  def contains?(rect, point)
    # x_min, x_max = rect.x, rect.x + rect.w
    # y_min, y_max = rect.y, rect.y + rect.h

    # point.x >= x_min &&
    #   point.x < x_max &&
    #   point.y >= y_min &&
    #   point.y < y_max

    point.x >= rect.x && point.x < rect.x + rect.w &&
        point.y >= rect.y && point.y < rect.y + rect.h
  end

  # Check if two rectangles intersect
  def intersects?(rect1, rect2)
    # x1_min, x1_max = rect1.x, rect1.x + rect1.w
    # y1_min, y1_max = rect1.y, rect1.y + rect1.h
    # x2_min, x2_max = rect2.x, rect2.x + rect2.w
    # y2_min, y2_max = rect2.y, rect2.y + rect2.h

    # !(x1_max <= x2_min || x1_min >= x2_max ||
    #     y1_max <= y2_min || y1_min >= y2_max)

    !(rect1.x + rect1.w <= rect2.x || rect1.x >= rect2.x + rect2.w ||
      rect1.y + rect1.h <= rect2.y || rect1.y >= rect2.y + rect2.h)
  end

  # Subdivide the quadtree into four quadrants
  def subdivide
    x, y, w, h = @bounds.x, @bounds.y, @bounds.w / 2, @bounds.h / 2

    @northwest ||= Quadtree.new({ x: x, y: y + h, w: w, h: h }, @capacity)
    @northeast ||= Quadtree.new({ x: x + w, y: y + h, w: w, h: h }, @capacity)
    @southwest ||= Quadtree.new({ x: x, y: y, w: w, h: h }, @capacity)
    @southeast ||= Quadtree.new({ x: x + w, y: y, w: w, h: h }, @capacity)

    @divided = true
  end
end
