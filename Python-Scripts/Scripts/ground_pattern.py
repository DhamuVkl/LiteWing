"""
PMW3901 Optical Flow Sensor Ground Pattern Generator
Generates optimal tracking patterns for position hold sensors
"""

from reportlab.lib.pagesizes import A4, letter
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm, cm
import random
import math

class PMW3901PatternGenerator:
    def __init__(self, filename="pmw3901_patterns.pdf", pagesize=A4):
        self.filename = filename
        self.pagesize = pagesize
        self.width, self.height = pagesize
        self.c = canvas.Canvas(filename, pagesize=pagesize)
        
    def add_title(self, title, subtitle=""):
        """Add title to current page"""
        self.c.setFont("Helvetica-Bold", 16)
        self.c.drawCentredString(self.width/2, self.height - 30, title)
        if subtitle:
            self.c.setFont("Helvetica", 10)
            self.c.drawCentredString(self.width/2, self.height - 45, subtitle)
    
    def pattern_random_circles(self):
        """Random circles pattern - excellent for optical flow"""
        self.c.showPage()
        self.add_title("Random Circles Pattern", 
                      "High contrast, non-repetitive - Excellent tracking")
        
        random.seed(42)  # For reproducibility
        y_start = self.height - 60
        
        for _ in range(1500):
            x = random.uniform(0, self.width)
            y = random.uniform(0, y_start)
            radius = random.uniform(3, 15)
            
            # Random fill or stroke
            if random.random() > 0.5:
                self.c.setFillColorRGB(0, 0, 0)
                self.c.circle(x, y, radius, fill=1, stroke=0)
            else:
                self.c.setStrokeColorRGB(0, 0, 0)
                self.c.setLineWidth(1.5)
                self.c.circle(x, y, radius, fill=0, stroke=1)
    
    def pattern_random_polygons(self):
        """Random polygons - great feature density"""
        self.c.showPage()
        self.add_title("Random Polygons Pattern",
                      "Variable shapes - Great feature tracking")
        
        random.seed(123)
        y_start = self.height - 60
        
        for _ in range(1000):
            cx = random.uniform(0, self.width)
            cy = random.uniform(0, y_start)
            num_sides = random.randint(3, 7)
            size = random.uniform(5, 20)
            
            path = self.c.beginPath()
            for i in range(num_sides):
                angle = 2 * math.pi * i / num_sides
                x = cx + size * math.cos(angle)
                y = cy + size * math.sin(angle)
                if i == 0:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
            path.close()
            
            self.c.setFillColorRGB(0, 0, 0)
            self.c.drawPath(path, fill=1, stroke=0)
    
    def pattern_checkerboard_irregular(self):
        """Irregular checkerboard - good baseline pattern"""
        self.c.showPage()
        self.add_title("Irregular Checkerboard",
                      "Variable sized squares - Good for orientation tracking")
        
        random.seed(456)
        y_start = self.height - 60
        
        y = 0
        while y < y_start:
            x = 0
            row_height = random.uniform(15, 40)
            
            while x < self.width:
                col_width = random.uniform(15, 40)
                
                if random.random() > 0.5:
                    self.c.setFillColorRGB(0, 0, 0)
                    self.c.rect(x, y, col_width, row_height, fill=1, stroke=0)
                
                x += col_width
            y += row_height
    
    def pattern_dots_grid_jittered(self):
        """Jittered dot grid - optimal for PMW3901"""
        self.c.showPage()
        self.add_title("Jittered Dot Grid",
                      "Semi-regular dots with position variation - Optimal balance")
        
        random.seed(789)
        y_start = self.height - 60
        spacing = 19
        jitter = 8
        
        y = spacing
        while y < y_start:
            x = spacing
            while x < self.width - spacing:
                jx = x + random.uniform(-jitter, jitter)
                jy = y + random.uniform(-jitter, jitter)
                radius = random.uniform(2, 6)
                
                self.c.setFillColorRGB(0, 0, 0)
                self.c.circle(jx, jy, radius, fill=1, stroke=0)
                
                x += spacing
            y += spacing
    
    def pattern_perlin_noise(self):
        """Pseudo-random noise pattern"""
        self.c.showPage()
        self.add_title("High Frequency Pattern",
                      "Dense random features - Maximum feature density")
        
        random.seed(101)
        y_start = self.height - 60
        
        for _ in range(5000):
            x = random.uniform(0, self.width)
            y = random.uniform(0, y_start)
            size = random.uniform(1, 8)
            
            shape_type = random.choice(['circle', 'square', 'line'])
            
            self.c.setFillColorRGB(0, 0, 0)
            if shape_type == 'circle':
                self.c.circle(x, y, size, fill=1, stroke=0)
            elif shape_type == 'square':
                self.c.rect(x-size/2, y-size/2, size, size, fill=1, stroke=0)
            else:
                angle = random.uniform(0, math.pi)
                dx = size * math.cos(angle)
                dy = size * math.sin(angle)
                self.c.line(x-dx, y-dy, x+dx, y+dy)
    
    def pattern_natural_stones(self):
        """Natural stone-like pattern"""
        self.c.showPage()
        self.add_title("Organic Stone Pattern",
                      "Natural irregular shapes - Good outdoor simulation")
        
        random.seed(202)
        y_start = self.height - 60
        
        for _ in range(400):
            cx = random.uniform(0, self.width)
            cy = random.uniform(0, y_start)
            num_points = random.randint(6, 12)
            base_radius = random.uniform(10, 30)
            
            path = self.c.beginPath()
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                radius = base_radius * random.uniform(0.7, 1.3)
                x = cx + radius * math.cos(angle)
                y = cy + radius * math.sin(angle)
                if i == 0:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
            path.close()
            
            self.c.setFillColorRGB(0, 0, 0)
            self.c.drawPath(path, fill=1, stroke=0)
    
    def add_recommendations_page(self):
        """Add recommendations and usage guide"""
        self.c.showPage()
        self.c.setFont("Helvetica-Bold", 18)
        self.c.drawCentredString(self.width/2, self.height - 40, 
                                "PMW3901 Pattern Recommendations")
        
        y = self.height - 80
        line_height = 16
        
        recommendations = [
            ("Best Patterns for PMW3901:", "Helvetica-Bold", 12),
            ("", "Helvetica", 10),
            ("1. Jittered Dot Grid - Best overall performance", "Helvetica", 11),
            ("   • Semi-regular spacing prevents repetition issues", "Helvetica", 10),
            ("   • High contrast dots are easy to track", "Helvetica", 10),
            ("   • Works well at various heights (10cm - 1m)", "Helvetica", 10),
            ("", "Helvetica", 10),
            ("2. Random Circles - Excellent for varied lighting", "Helvetica", 11),
            ("   • Non-repetitive pattern", "Helvetica", 10),
            ("   • Good feature density", "Helvetica", 10),
            ("", "Helvetica", 10),
            ("3. High Frequency Pattern - Maximum precision", "Helvetica", 11),
            ("   • Best for low altitude (< 30cm)", "Helvetica", 10),
            ("   • Dense features for fine movement detection", "Helvetica", 10),
            ("", "Helvetica", 10),
            ("General Guidelines:", "Helvetica-Bold", 12),
            ("", "Helvetica", 10),
            ("• Print Pattern: Use matte finish to avoid reflections", "Helvetica", 10),
            ("• Contrast: Black on white works best", "Helvetica", 10),
            ("• Avoid: Perfectly regular grids or repetitive patterns", "Helvetica", 10),
            ("• Scale: Features should be 3-20mm for optimal tracking", "Helvetica", 10),
            ("• Lighting: Ensure even illumination, avoid shadows", "Helvetica", 10),
            ("• Height: Sensor works best 10cm - 80cm above pattern", "Helvetica", 10),
        ]
        
        for text, font, size in recommendations:
            self.c.setFont(font, size)
            self.c.drawString(50, y, text)
            y -= line_height
    
    def generate_all_patterns(self):
        """Generate all patterns and save PDF"""
        print("Generating PMW3901 ground patterns...")
        
        # Add recommendations first
        self.add_recommendations_page()
        
        # Generate all patterns
        self.pattern_dots_grid_jittered()
        self.pattern_random_circles()
        self.pattern_random_polygons()
        self.pattern_checkerboard_irregular()
        self.pattern_perlin_noise()
        self.pattern_natural_stones()
        
        # Save PDF
        self.c.save()
        print(f"✓ PDF generated: {self.filename}")
        print(f"  Total pages: 7 (1 guide + 6 patterns)")
        print(f"\nRecommended patterns:")
        print("  1. Jittered Dot Grid (page 2)")
        print("  2. Random Circles (page 3)")
        print("  3. High Frequency Pattern (page 6)")

# Generate the PDF
if __name__ == "__main__":
    generator = PMW3901PatternGenerator("pmw3901_tracking_patterns.pdf", pagesize=A4)
    generator.generate_all_patterns()