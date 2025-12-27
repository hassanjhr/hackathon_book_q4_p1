# Feature Specification: Navbar & Auth UI Cleanup (Phase 4)

**Feature ID:** 004-navbar-auth-ui-cleanup
**Branch:** 004-navbar-auth-ui-cleanup
**Priority:** HIGH
**Type:** UI Enhancement (Frontend Only)
**Date:** 2025-12-17

---

## Overview

Clean up and simplify the Docusaurus navbar by removing redundant module links and adding a single user authentication icon with auth pages (UI only, no backend implementation).

---

## Scope

### In Scope
- Docusaurus navbar configuration updates
- Custom React components for auth UI
- Static auth pages (Login, Signup)
- User icon with auth choice modal/page
- Clean, centered UI designs

### Out of Scope
- Authentication backend logic
- API integration
- Database implementation
- User session management
- New npm dependencies (use only existing Docusaurus capabilities)
- Modification of existing markdown content

---

## Goals

1. **Simplify Navigation**: Remove clutter from top navbar (Module 2, 3, 4)
2. **Single Auth Entry Point**: Unified user icon for all auth flows
3. **Professional UI**: Clean, centered auth pages matching Docusaurus theme
4. **Production Ready**: Build passes without errors, no backend dependencies

---

## User Stories

### US-1: Simplified Navbar (User)
**As a** visitor
**I want** a clean navbar without repeated module links
**So that** navigation is simpler and I can find content via sidebar

**Acceptance Criteria:**
- Top navbar shows only: Logo, "Learning" dropdown
- Right side shows: Language switch, GitHub, Theme toggle, User icon
- Module 2, 3, 4 removed from top navbar
- All modules still accessible via sidebar

---

### US-2: Auth Entry Point (User)
**As a** visitor
**I want** a single user icon to access login/signup
**So that** I don't see multiple auth options cluttering the UI

**Acceptance Criteria:**
- User icon (👤) appears in top right navbar
- Clicking icon navigates to `/auth` choice page or opens modal
- Choice page shows "Welcome!" title
- Two clear buttons: "Sign In" and "Sign Up"

---

### US-3: Login Page (User)
**As a** returning user
**I want** a login page
**So that** I can prepare for future authentication

**Acceptance Criteria:**
- Route: `/login`
- Clean, centered UI
- Fields: Email, Password
- Submit button: "Login"
- Notice: "Authentication coming soon"
- Matches Docusaurus theme (light/dark mode)

---

### US-4: Signup Page (User)
**As a** new user
**I want** a signup page
**So that** I can prepare for future account creation

**Acceptance Criteria:**
- Route: `/signup`
- Clean, centered UI
- Fields: Name, Email, Password
- Submit button: "Create Account"
- Notice: "Signup will be enabled soon"
- Matches Docusaurus theme (light/dark mode)

---

## Functional Requirements

### FR-1: Navbar Configuration
- **FR-1.1**: Remove module links from `docusaurus.config.ts` navbar items
  - Keep: Logo, "Learning" dropdown
  - Remove: Individual module links (Module 2, 3, 4)

- **FR-1.2**: Add user icon component to navbar
  - Position: Right side, after theme toggle
  - Icon: User/account icon
  - Behavior: Navigate to `/auth` or trigger modal

### FR-2: Auth Choice Page
- **FR-2.1**: Create `/auth` route page
  - Title: "Welcome!"
  - Subtitle: "Choose an option to continue"

- **FR-2.2**: Two action buttons
  - Button 1: "🔐 Sign In" → Navigate to `/login`
  - Button 2: "✨ Sign Up" → Navigate to `/signup`

- **FR-2.3**: Responsive centered layout
  - Works on mobile, tablet, desktop
  - Matches Docusaurus theme

### FR-3: Login Page Component
- **FR-3.1**: Create React component at `/src/pages/login.tsx`
  - Form fields: Email (type: email), Password (type: password)
  - Submit button: "Login"
  - Disabled state (no backend)

- **FR-3.2**: Display notice
  - Text: "Authentication is coming soon. This page is for demonstration purposes."
  - Styled as info alert

- **FR-3.3**: Responsive design
  - Centered on desktop
  - Full width on mobile with padding
  - Theme-aware (light/dark mode)

### FR-4: Signup Page Component
- **FR-4.1**: Create React component at `/src/pages/signup.tsx`
  - Form fields: Name (text), Email (email), Password (password)
  - Submit button: "Create Account"
  - Disabled state (no backend)

- **FR-4.2**: Display notice
  - Text: "Signup will be enabled soon. Stay tuned!"
  - Styled as info alert

- **FR-4.3**: Responsive design
  - Same responsive patterns as login page
  - Theme-aware

---

## Non-Functional Requirements

### NFR-1: Performance
- No new npm dependencies (use Docusaurus built-ins)
- Static pages (no API calls)
- Fast load times (<100ms for auth pages)

### NFR-2: Compatibility
- Works in all modern browsers (Chrome, Firefox, Safari, Edge)
- Mobile responsive (down to 320px width)
- Supports both light and dark themes

### NFR-3: Build Safety
- `npm run build` must pass without errors
- No TypeScript errors
- No React warnings in console
- Static site generation succeeds

### NFR-4: Accessibility
- Keyboard navigable
- ARIA labels for icons
- Form labels associated with inputs
- Color contrast meets WCAG AA standards

---

## Technical Requirements

### TR-1: Docusaurus Configuration
- **File**: `my-website/docusaurus.config.ts`
- **Changes**:
  - Update `navbar.items` array
  - Remove: Module 2, 3, 4 links
  - Add: Custom user icon component reference

### TR-2: Custom React Components
- **Location**: `my-website/src/components/`
- **Components**:
  - `UserIcon.tsx`: Navbar user icon with click handler
  - Optional: `AuthModal.tsx` (if using modal instead of page)

### TR-3: Static Pages
- **Location**: `my-website/src/pages/`
- **Pages**:
  - `auth.tsx`: Auth choice page
  - `login.tsx`: Login form page
  - `signup.tsx`: Signup form page

### TR-4: Styling
- Use Docusaurus CSS modules or Infima variables
- No external CSS libraries
- Theme CSS variables for consistency:
  - `--ifm-color-primary`
  - `--ifm-background-color`
  - `--ifm-font-color-base`

---

## User Interface Mockups

### Navbar (After Cleanup)
```
┌────────────────────────────────────────────────────────────┐
│ [Logo] AI Robotics  [Learning ▼]     🌐 [GitHub] 🌙 👤    │
└────────────────────────────────────────────────────────────┘
```

### Auth Choice Page (`/auth`)
```
┌──────────────────────────────────────┐
│                                      │
│           Welcome! 👋                 │
│    Choose an option to continue      │
│                                      │
│   ┌──────────────────────────┐      │
│   │   🔐 Sign In             │      │
│   └──────────────────────────┘      │
│                                      │
│   ┌──────────────────────────┐      │
│   │   ✨ Sign Up             │      │
│   └──────────────────────────┘      │
│                                      │
└──────────────────────────────────────┘
```

### Login Page (`/login`)
```
┌──────────────────────────────────────┐
│                                      │
│           Login 🔐                    │
│                                      │
│   Email                              │
│   ┌────────────────────────────┐    │
│   │                            │    │
│   └────────────────────────────┘    │
│                                      │
│   Password                           │
│   ┌────────────────────────────┐    │
│   │ ••••••••••                 │    │
│   └────────────────────────────┘    │
│                                      │
│   ┌────────────────────────────┐    │
│   │        Login               │    │
│   └────────────────────────────┘    │
│                                      │
│   ℹ️ Authentication coming soon      │
│                                      │
└──────────────────────────────────────┘
```

### Signup Page (`/signup`)
```
┌──────────────────────────────────────┐
│                                      │
│        Create Account ✨              │
│                                      │
│   Name                               │
│   ┌────────────────────────────┐    │
│   │                            │    │
│   └────────────────────────────┘    │
│                                      │
│   Email                              │
│   ┌────────────────────────────┐    │
│   │                            │    │
│   └────────────────────────────┘    │
│                                      │
│   Password                           │
│   ┌────────────────────────────┐    │
│   │ ••••••••••                 │    │
│   └────────────────────────────┘    │
│                                      │
│   ┌────────────────────────────┐    │
│   │   Create Account           │    │
│   └────────────────────────────┘    │
│                                      │
│   ℹ️ Signup will be enabled soon     │
│                                      │
└──────────────────────────────────────┘
```

---

## Data Model

**None required** - This is a frontend-only feature with no backend data persistence.

---

## API Contracts

**None required** - No API calls, no backend integration.

---

## Dependencies

### Existing (No New Dependencies)
- React (via Docusaurus)
- TypeScript (via Docusaurus)
- Docusaurus 3.x
- @docusaurus/theme-classic

### Files to Modify
1. `my-website/docusaurus.config.ts` - Navbar configuration
2. (Create) `my-website/src/components/UserIcon.tsx` - User icon component
3. (Create) `my-website/src/pages/auth.tsx` - Auth choice page
4. (Create) `my-website/src/pages/login.tsx` - Login page
5. (Create) `my-website/src/pages/signup.tsx` - Signup page

---

## Testing Criteria

### Manual Testing
- ✅ Navbar displays correctly (no Module 2, 3, 4)
- ✅ User icon appears in top right
- ✅ Clicking user icon navigates to `/auth`
- ✅ Auth choice page shows two buttons
- ✅ "Sign In" button navigates to `/login`
- ✅ "Sign Up" button navigates to `/signup`
- ✅ Login page renders with email/password fields
- ✅ Signup page renders with name/email/password fields
- ✅ All pages work in light and dark mode
- ✅ Mobile responsive (test at 320px, 768px, 1024px widths)
- ✅ Build passes: `npm run build`

### Accessibility Testing
- ✅ Tab navigation works through all forms
- ✅ Screen reader announces form labels
- ✅ Color contrast passes WCAG AA
- ✅ Focus indicators visible

---

## Risks and Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Build breaks due to TypeScript errors | HIGH | LOW | Use strict TypeScript, test build frequently |
| Theme inconsistency (light/dark) | MEDIUM | MEDIUM | Use Docusaurus CSS variables exclusively |
| Mobile layout breaks | MEDIUM | LOW | Test responsive design early, use Docusaurus grid system |
| Accessibility issues | MEDIUM | LOW | Follow WCAG guidelines, test with keyboard navigation |

---

## Implementation Phases

### Phase 1: Navbar Cleanup (30 mins)
- Remove module links from `docusaurus.config.ts`
- Test build
- Verify sidebar navigation still works

### Phase 2: User Icon Component (30 mins)
- Create `UserIcon.tsx` component
- Add to navbar configuration
- Test click navigation to `/auth`

### Phase 3: Auth Pages (1 hour)
- Create `/auth` choice page
- Create `/login` page
- Create `/signup` page
- Style with Docusaurus theme variables

### Phase 4: Testing & Refinement (30 mins)
- Test all pages in light/dark mode
- Test mobile responsive layouts
- Run full build
- Fix any TypeScript/build errors

**Total Estimated Time:** 2-3 hours

---

## Success Criteria

✅ Navbar shows only Logo, Learning, and right-side utilities
✅ User icon navigates to auth choice page
✅ Login and signup pages render correctly
✅ All pages support light/dark themes
✅ Mobile responsive down to 320px
✅ Build passes without errors
✅ No new npm dependencies added
✅ Keyboard navigation works
✅ No markdown content modified

---

## Out of Scope (Future Work)

- Actual authentication backend
- User session management
- OAuth providers (Google, GitHub)
- Password reset functionality
- Email verification
- User profile pages
- Protected routes

---

## References

- [Docusaurus Configuration](https://docusaurus.io/docs/configuration)
- [Docusaurus Navbar](https://docusaurus.io/docs/api/themes/configuration#navbar)
- [Docusaurus Pages](https://docusaurus.io/docs/creating-pages)
- [Infima CSS Variables](https://infima.dev/docs/getting-started/introduction)

---

**Status:** Ready for Planning
**Next Step:** Create implementation plan with technical design
