package edu.edina.Libraries.Actions;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

// Indicates that an action will take control of the part named "part",
// and that no other action should simultaneously try to control that part.
// You can use whatever string you want for part, but keep it simple: (like
// "drivetrain", "claw", etc.) because you have to use it consistently.

@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
@Repeatable(value = ArrayOfControls.class)
public @interface Controls {
    String part();
}
