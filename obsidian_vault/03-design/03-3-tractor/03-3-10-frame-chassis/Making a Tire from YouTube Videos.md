Tire video: [Fusion 360 - (1/3) Parametric Tire and Treads for Beginners/Intermediate Users - Lesson 12 (2023)](https://youtu.be/EbPbMoG6ltY?si=NzpUSyEuUDEiEx3H&t=1021)
 ![Embedded YouTube video](https://www.youtube.com/embed/EbPbMoG6ltY?start=1021&feature=oembed&autoplay=true)  

Create an offset plane (tireOuterDia/2)+1

![[Exported image 20260511102724-0.png]]  

Sketch a center point rectangle on the offset plane  
Width: TireWidth; Length: ( tireOuterDia * PI ) / treadPatternNo
 
@ 12:45 sketch the tread to be embossed on the tire  
Add center line construction lines  
Add two vertical lines with symmetrical constraint on the center line with treadGap as distance

![[Exported image 20260511102726-1.png]]

Add a vertical line on the right with a treadGap/2 from the edge
 
Add a symmetrical line in the middle of the box on the right side￼

![[Exported image 20260511102727-2.png]]

Add two vertical lines symmetrical with the new construction line

![[Exported image 20260511102730-3.png]]

Create this pattern

![[Exported image 20260511102731-4.png]]

Draw a chevron on both sides and add dimensions to constrain the lines.

![[Exported image 20260511102741-5.png]]  

For the right edge add co-linear lines to extend the image  
Add vertical constraint on edge line  
Dimension the right most line to treadGap*1.5  
Make lines on the right construction lines to enlarge the profile

![[Exported image 20260511102743-6.png]]   
Create, Emboss  
Use treadHeight for depth

![[Exported image 20260511102745-7.png]]  

Result

![[Exported image 20260511102746-8.png]]  

Mirror the new body  
Mirror; Object Type Features; XZ plane (the right view, center of the wheel)

![[Exported image 20260511102748-9.png]]  

Add circular pattern  
Create; Pattern; Circular Patter  
Features;  
Quantity: treadPatternNo

![[Exported image 20260511102749-10.png]]  

Add outside cut - 27:00

- Open outside rim sketch
- Create a 3 pt arc
- ![[Exported image 20260511102750-11.png]]
- Add two tangent constraints
- ![[Exported image 20260511102755-12.png]]
- Add another 3 pt arc
- ![[Exported image 20260511102757-13.png]]
- Make it a 2" radius
- Finish Sketch
- Make the sketch visible
- ![[Exported image 20260511102758-14.png]] - ![[Exported image 20260511102800-15.png]]
- Mirror to the other side
- Mirror; Features; Select Profile; Axis is the XZ Plane; Adjust
- ![[Exported image 20260511102802-16.png]]

|   |   |
|---|---|
|![[Exported image 20260511102804-17.png]]|![[Exported image 20260511102805-18.png]]|
   

Make the rim
 
|   |   |
|---|---|
|[How To Model a Wheel in Fusion 360 - Day 10](https://www.youtube.com/watch?v=2-MjfWE8mmY)<br><br>  <br>![Embedded YouTube video](https://www.youtube.com/embed/2-MjfWE8mmY?feature=oembed&autoplay=true)|![[Exported image 20260511102809-19.png]]|

Add 'rim' as component  
Create sketch and select 'slice' to show the outline of the tire

![[Exported image 20260511102810-20.png]]  

Add horizontal, construction line with a midpoint constraint; Make the width 'rimWidth'
 
1"; 2.421"; 2.15"

![[Making a Tire from YouTube Videos - Ink.svg]]
