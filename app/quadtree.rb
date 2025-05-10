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
    @objects.each do |object|
      found << object if contains?(range, object)
    end

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

  def draw_outputs(args)
    # Draw the objects in the quadtree
    @objects.each do |object|
      args.outputs.solids << { x: object.x, y: object.y, w: 5, h: 5, r: 255, g: 0, b: 0 }
    end
    
    # Visualize the quadtree for debugging
    args.outputs.borders << { x: @bounds.x, y: @bounds.y, w: @bounds.w, h: @bounds.h }
    if @divided
      @northwest.draw_outputs(args)
      @northeast.draw_outputs(args)
      @southwest.draw_outputs(args)
      @southeast.draw_outputs(args)
    end
  end

  private

  # Check if a point lies within a rectangle
  def contains?(rect, point)
    point.x >= rect.x &&
      point.x < rect.x + rect.w &&
      point.y >= rect.y &&
      point.y < rect.y + rect.h
  end

  # Check if two rectangles intersect
  def intersects?(rect1, rect2)
    !(rect1.x + rect1.w <= rect2.x || rect1.x >= rect2.x + rect2.w ||
      rect1.y + rect1.h <= rect2.y || rect1.y >= rect2.y + rect2.h)
  end

  # Subdivide the quadtree into four quadrants
  def subdivide
    x, y, w, h = @bounds.x, @bounds.y, @bounds.w / 2, @bounds.h / 2

    @northwest = Quadtree.new({ x: x, y: y + h, w: w, h: h }, @capacity)
    @northeast = Quadtree.new({ x: x + w, y: y + h, w: w, h: h }, @capacity)
    @southwest = Quadtree.new({ x: x, y: y, w: w, h: h }, @capacity)
    @southeast = Quadtree.new({ x: x + w, y: y, w: w, h: h }, @capacity)

    @divided = true
  end
end
