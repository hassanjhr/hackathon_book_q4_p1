# Research: Navbar & Auth UI Cleanup

**Feature:** 004-navbar-auth-ui-cleanup
**Date:** 2025-12-17
**Phase:** 0 (Research & Technical Investigation)

---

## Research Summary

This document consolidates research findings for implementing navbar cleanup and auth UI pages in Docusaurus 3.9.2.

---

## Decision 1: Navbar Link Removal Strategy

**Decision:** Use direct configuration modification in `docusaurus.config.ts`

**Rationale:**
- Simplest and most maintainable approach
- No component swizzling required
- Changes are explicit in configuration
- No risk of breaking changes in future Docusaurus versions

**Implementation:**
```typescript
// Remove from navbar.items array:
// - Module 2, 3, 4 individual links
// Keep only: Logo, "Learning" dropdown, right-side utilities
```

**Alternatives Considered:**
1. ❌ Dynamic filtering with JavaScript - Unnecessary complexity
2. ❌ Conditional rendering with environment variables - Overkill for static removal

**References:**
- [Theme Configuration | Docusaurus](https://docusaurus.io/docs/api/themes/configuration)

---

## Decision 2: Custom User Icon Component Approach

**Decision:** Use custom navbar component type with `custom-` prefix pattern

**Rationale:**
- Official Docusaurus pattern (issue #7227)
- No swizzling required (maintains upgrade path)
- Theme-aware by default
- Clean separation of concerns

**Implementation Pattern:**
```typescript
// 1. Create component: src/components/NavbarItems/UserIconNavbarItem.tsx
// 2. Register type: src/theme/NavbarItem/ComponentTypes.tsx
// 3. Configure: docusaurus.config.ts with type: 'custom-userIcon'
```

**Key Technical Requirements:**
- Component must accept `mobile?: boolean` prop for responsive behavior
- Must use `@docusaurus/Link` for routing (not `<a>` tags)
- Must use CSS custom properties for theme compatibility
- Must include ARIA labels for accessibility

**Alternatives Considered:**
1. ❌ Full navbar swizzling - Too invasive, maintenance burden
2. ❌ Browser extension/script injection - Not production-ready
3. ❌ CSS-only solution - Cannot handle click events

**References:**
- [GitHub Issue #7227: Custom navbar components](https://github.com/facebook/docusaurus/issues/7227)
- [Swizzling Guide](https://docusaurus.io/docs/swizzling)

---

## Decision 3: Auth Pages Implementation

**Decision:** Use static React pages in `src/pages/` directory with client-side-only form handling

**Rationale:**
- Docusaurus file-based routing works out of the box
- TypeScript support included
- Theme variables automatically available
- No backend required (matches project scope)
- SSR-safe with proper patterns

**Implementation Structure:**
```
src/pages/
├── auth.tsx          # Auth choice page (/auth route)
├── login.tsx         # Login form (/login route)
└── signup.tsx        # Signup form (/signup route)
```

**Form Handling Strategy:**
- Forms will be **non-functional** (no backend integration)
- Display notice: "Authentication coming soon"
- Use React state for UI feedback (validation, button states)
- No external form services (Formspree, etc.) needed

**Alternatives Considered:**
1. ❌ MDX pages - Less flexible for complex interactions
2. ❌ Modal-based auth - Harder to share direct links
3. ❌ External auth service integration - Out of scope

**References:**
- [Creating Pages | Docusaurus](https://docusaurus.io/docs/creating-pages)

---

## Decision 4: Styling Approach

**Decision:** Use CSS Modules + Infima CSS variables

**Rationale:**
- Component-scoped styling (no global conflicts)
- Automatic theme compatibility with Infima variables
- No additional dependencies required
- Follows Docusaurus best practices
- Mobile-responsive by default with Infima grid

**Key CSS Variables to Use:**
```css
/* Colors */
--ifm-color-primary
--ifm-color-primary-dark
--ifm-navbar-link-color
--ifm-navbar-link-hover-color
--ifm-background-color
--ifm-card-background-color

/* Typography */
--ifm-font-color-base
--ifm-heading-color
--ifm-font-family-base

/* Spacing */
--ifm-spacing-horizontal
--ifm-spacing-vertical
```

**Mobile Breakpoint:** 996px (Docusaurus standard)

**Alternatives Considered:**
1. ❌ Tailwind CSS - New dependency, conflicts with Infima
2. ❌ Styled Components - Runtime overhead, not Docusaurus pattern
3. ❌ Global CSS only - No scoping, maintenance issues

**References:**
- [Styling and Layout | Docusaurus](https://docusaurus.io/docs/styling-layout)
- [Infima CSS Variables](https://docusaurus.community/knowledge/design/css/variables/)

---

## Decision 5: Theme Compatibility Implementation

**Decision:** CSS-only theme detection (no `useColorMode` hook)

**Rationale:**
- Avoids hydration mismatches (SSR issue)
- Better performance (no JavaScript overhead)
- Simpler implementation
- Recommended by Docusaurus docs

**Pattern:**
```css
.element {
  color: var(--ifm-font-color-base); /* Automatic theme switching */
}

/* Explicit dark mode override if needed */
[data-theme='dark'] .element {
  background: #1e293b;
}
```

**When to Use `useColorMode` Hook:**
- Only when JavaScript logic depends on theme state
- Not needed for styling (use CSS variables instead)

**Alternatives Considered:**
1. ❌ `useColorMode` hook everywhere - Causes SSR/hydration issues
2. ❌ Separate light/dark components - Code duplication
3. ❌ JavaScript-based theme detection - Unnecessary complexity

**References:**
- [How to Detect Docusaurus Light or Dark Mode](https://www.codestudy.net/blog/is-it-possible-to-detect-if-docusaurus-is-in-light-or-dark-mode/)

---

## Decision 6: Routing Strategy

**Decision:** Use `@docusaurus/Link` component for all navigation

**Rationale:**
- Built-in preloading for better performance
- Proper React Router integration
- Prevents full page reloads
- Handles `baseUrl` configuration automatically
- TypeScript support included

**Pattern:**
```typescript
import Link from '@docusaurus/Link';

<Link to="/auth" className="navbar__link">
  <UserIcon />
</Link>
```

**Alternatives Considered:**
1. ❌ Native `<a>` tags - Full page reload, loses SPA benefits
2. ❌ `useHistory` hook - More complex, unnecessary for simple links
3. ❌ `window.location` - Causes page reload

**References:**
- [@docusaurus/Link API](https://docusaurus.io/docs/docusaurus-core#link)

---

## Technical Specifications Resolved

### 1. Navbar Configuration Changes

**File:** `my-website/docusaurus.config.ts`

**Current Navbar Items (to be removed):**
```typescript
{
  to: '/docs/module-2/intro',
  label: 'Module 2',
  position: 'left',
},
{
  to: '/docs/module-3/intro',
  label: 'Module 3',
  position: 'left',
},
{
  to: '/docs/module-4/intro',
  label: 'Module 4',
  position: 'left',
},
```

**New Navbar Structure:**
```typescript
navbar: {
  title: 'Physical AI Textbook',
  logo: { /* existing */ },
  items: [
    // LEFT SIDE
    {
      type: 'docSidebar',
      sidebarId: 'tutorialSidebar',
      position: 'left',
      label: 'Learning',
    },

    // RIGHT SIDE
    {
      type: 'localeDropdown',
      position: 'right',
    },
    {
      href: 'https://github.com/hassanjhr/hackathon_book_q4_p1',
      label: 'GitHub',
      position: 'right',
    },
    {
      type: 'html',
      position: 'right',
      value: '<div class="navbar-divider"></div>',
    },
    {
      type: 'custom-userIcon',  // NEW CUSTOM COMPONENT
      position: 'right',
      authUrl: '/auth',
    },
  ],
}
```

---

### 2. Custom Component Structure

**UserIcon Component:**
- **Location:** `src/components/NavbarItems/UserIconNavbarItem.tsx`
- **Props Interface:**
  ```typescript
  interface UserIconNavbarItemProps {
    mobile?: boolean;
    authUrl?: string;
  }
  ```
- **Icon:** SVG user icon (Material Design style)
- **Behavior:** Navigate to `/auth` on click
- **Mobile:** Show icon only (no text label)
- **Desktop:** Show icon + optional label

**Component Registration:**
- **Location:** `src/theme/NavbarItem/ComponentTypes.tsx`
- **Pattern:** Wrap original `ComponentTypes` and add custom type

---

### 3. Page Component Specifications

#### Auth Choice Page (`/auth`)
- **Route:** `/auth`
- **Layout:** Centered card with max-width 500px
- **Content:**
  - Heading: "Welcome!"
  - Subheading: "Choose an option to continue"
  - Button 1: "Sign In" → `/login`
  - Button 2: "Sign Up" → `/signup`
- **Styling:** Card with shadow, responsive padding

#### Login Page (`/login`)
- **Route:** `/login`
- **Fields:**
  - Email (type: email, required)
  - Password (type: password, required)
- **Button:** "Login" (disabled by default)
- **Notice:** Info alert with "Authentication coming soon"
- **Layout:** Centered form, max-width 400px

#### Signup Page (`/signup`)
- **Route:** `/signup`
- **Fields:**
  - Name (type: text, required)
  - Email (type: email, required)
  - Password (type: password, required)
- **Button:** "Create Account" (disabled by default)
- **Notice:** Info alert with "Signup will be enabled soon"
- **Layout:** Centered form, max-width 400px

---

### 4. Styling Standards

**CSS Module Naming Convention:**
```
[ComponentName].module.css
```

**Required Responsive Breakpoints:**
```css
/* Mobile-first base styles */
.element { }

/* Tablet (optional) */
@media screen and (min-width: 768px) { }

/* Desktop (Docusaurus standard) */
@media screen and (min-width: 996px) { }
```

**Required Theme Variables:**
- Background: `var(--ifm-background-color)`
- Text: `var(--ifm-font-color-base)`
- Primary: `var(--ifm-color-primary)`
- Cards: `var(--ifm-card-background-color)`
- Borders: `var(--ifm-color-emphasis-300)`

---

### 5. Accessibility Requirements

**ARIA Labels:**
- User icon: `aria-label="User menu"`
- Form fields: Explicit `<label for="...">` associations
- Buttons: Descriptive text or `aria-label`

**Keyboard Navigation:**
- All clickable elements focusable via Tab
- Enter/Space activates buttons and links
- Escape closes modals (if implemented later)

**Color Contrast:**
- WCAG AA compliance (4.5:1 for normal text)
- Infima variables handle this automatically

---

## Dependencies Analysis

**Existing Dependencies (No New Additions Required):**
- ✅ React 18.x (via Docusaurus)
- ✅ TypeScript (via Docusaurus)
- ✅ @docusaurus/theme-classic
- ✅ @docusaurus/Link
- ✅ @docusaurus/router
- ✅ clsx (for conditional classes)

**No External Libraries Needed For:**
- Form handling (React state only)
- Routing (Docusaurus built-in)
- Styling (CSS Modules + Infima)
- Icons (SVG inline)
- Theme detection (CSS variables)

---

## Build Safety Validation

**Required Checks:**
1. ✅ TypeScript compilation passes
2. ✅ No React warnings in console
3. ✅ Static site generation succeeds
4. ✅ All routes accessible
5. ✅ Mobile responsive (320px minimum)
6. ✅ Light/dark theme switching works
7. ✅ Navbar renders correctly on all pages

**Test Command:**
```bash
cd my-website && npm run build
```

**Expected Output:**
```
[SUCCESS] Generated static files in "build".
```

---

## Performance Considerations

**Optimization Strategies:**
- Use CSS Modules for automatic code splitting
- Inline SVG icons (no external requests)
- No JavaScript for theme detection (CSS only)
- Leverage Docusaurus Link preloading
- Minimal component re-renders (React.memo if needed)

**Expected Impact:**
- Navbar: <1KB additional JavaScript
- Auth pages: ~5KB total (TSX + CSS)
- No impact on existing doc pages
- No additional network requests

---

## Risk Mitigation

| Risk | Mitigation |
|------|-----------|
| TypeScript errors in custom components | Follow existing project patterns, use strict types |
| Theme inconsistency | Use only Infima CSS variables, test both themes |
| Mobile layout breaks | Mobile-first CSS, test at 320px width |
| Build failures | Test build after each file creation |
| Navbar positioning issues | Use Docusaurus standard navbar classes |
| Routing conflicts | Use Docusaurus Link component exclusively |

---

## Research Completion Checklist

- ✅ Navbar customization strategy defined
- ✅ Custom component pattern researched
- ✅ Page routing approach validated
- ✅ Styling methodology selected
- ✅ Theme compatibility solution identified
- ✅ Accessibility requirements documented
- ✅ Performance impact assessed
- ✅ No new dependencies required
- ✅ Build safety validation planned
- ✅ All technical unknowns resolved

---

## Next Steps

**Phase 1: Design**
1. Create data-model.md (minimal - no backend data)
2. Create contracts/ (none required - frontend only)
3. Create quickstart.md (integration guide)
4. Generate tasks.md with implementation breakdown

**Phase 2: Implementation**
1. Navbar configuration changes
2. UserIcon component creation
3. Auth page components
4. CSS Module styling
5. Testing and validation

---

## References

1. [Docusaurus Theme Configuration](https://docusaurus.io/docs/api/themes/configuration)
2. [Creating Pages | Docusaurus](https://docusaurus.io/docs/creating-pages)
3. [Styling and Layout | Docusaurus](https://docusaurus.io/docs/styling-layout)
4. [GitHub Issue #7227: Custom navbar items](https://github.com/facebook/docusaurus/issues/7227)
5. [Infima CSS Variables](https://docusaurus.community/knowledge/design/css/variables/)
6. [Swizzling Guide](https://docusaurus.io/docs/swizzling)

---

**Status:** ✅ Research Complete
**Ready for:** Phase 1 (Design)
**All Unknowns Resolved:** YES
**New Dependencies:** NONE
